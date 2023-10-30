/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "adBms_Application.h"
#include "my_adc_dma.h"
#include "my_define.h"
#include "my_function.h"
#include "gpio.h"
#include "spi.h"
#include "Can_function.h"
#include "my_define.h"
#include "usart.h"
#include "EKF.h"
#include "math.h"
#include "Serial_function.h"
#include "Serial485_function.h"
#include "tim.h"
#include "adc.h"
#include "adc.h"
#include "i2c.h"
#include "bms.h"

//#include "SocAlgorithm.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PRE_CHARGE_CHECK_Pin GPIO_PIN_13
#define PRE_CHARGE_CHECK_GPIO_Port GPIOC
#define CHARGE_PWR_EN_Pin GPIO_PIN_14
#define CHARGE_PWR_EN_GPIO_Port GPIOC
#define DISCHARGE_PWR_EN_Pin GPIO_PIN_15
#define DISCHARGE_PWR_EN_GPIO_Port GPIOC
#define aBAT_CURRENT_Pin GPIO_PIN_1
#define aBAT_CURRENT_GPIO_Port GPIOC
#define aNTC1_Pin GPIO_PIN_2
#define aNTC1_GPIO_Port GPIOC
#define aNTC0_Pin GPIO_PIN_3
#define aNTC0_GPIO_Port GPIOC
#define aCT_SENSE_Pin GPIO_PIN_1
#define aCT_SENSE_GPIO_Port GPIOA
#define MCU_TX_Pin GPIO_PIN_2
#define MCU_TX_GPIO_Port GPIOA
#define MCU_RX_Pin GPIO_PIN_3
#define MCU_RX_GPIO_Port GPIOA
#define SPI1_nCS_Pin GPIO_PIN_4
#define SPI1_nCS_GPIO_Port GPIOA
#define CHARGER_EN_CHECK_Pin GPIO_PIN_5
#define CHARGER_EN_CHECK_GPIO_Port GPIOC
#define FAN_LOW_SPEED_Pin GPIO_PIN_0
#define FAN_LOW_SPEED_GPIO_Port GPIOB
#define PRE_CHARGE_RELAY_EN_Pin GPIO_PIN_1
#define PRE_CHARGE_RELAY_EN_GPIO_Port GPIOB
#define BOOT1_Pin GPIO_PIN_2
#define BOOT1_GPIO_Port GPIOB
#define SCL_Pin GPIO_PIN_10
#define SCL_GPIO_Port GPIOB
#define SDA_Pin GPIO_PIN_11
#define SDA_GPIO_Port GPIOB
#define CAN_ID1_Pin GPIO_PIN_12
#define CAN_ID1_GPIO_Port GPIOB
#define CAN_ID2_Pin GPIO_PIN_13
#define CAN_ID2_GPIO_Port GPIOB
#define CAN_ID4_Pin GPIO_PIN_14
#define CAN_ID4_GPIO_Port GPIOB
#define CAN_ID8_Pin GPIO_PIN_15
#define CAN_ID8_GPIO_Port GPIOB
#define MOTOR_RELAY_PWM_Pin GPIO_PIN_6
#define MOTOR_RELAY_PWM_GPIO_Port GPIOC
#define EX_CHARG_RELAY_PWM_Pin GPIO_PIN_7
#define EX_CHARG_RELAY_PWM_GPIO_Port GPIOC
#define SMPS_RELAY_PWM_Pin GPIO_PIN_8
#define SMPS_RELAY_PWM_GPIO_Port GPIOC
#define FAN_HIGH_SPEED_PWM_Pin GPIO_PIN_9
#define FAN_HIGH_SPEED_PWM_GPIO_Port GPIOC
#define COM_SEL0_Pin GPIO_PIN_8
#define COM_SEL0_GPIO_Port GPIOA
#define DEBUG_RX_Pin GPIO_PIN_9
#define DEBUG_RX_GPIO_Port GPIOA
#define DEBUG_TX_Pin GPIO_PIN_10
#define DEBUG_TX_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define COM_SEL1_Pin GPIO_PIN_15
#define COM_SEL1_GPIO_Port GPIOA
#define RS485_TX_Pin GPIO_PIN_10
#define RS485_TX_GPIO_Port GPIOC
#define RS485_RX_Pin GPIO_PIN_11
#define RS485_RX_GPIO_Port GPIOC
#define MOTOR_RELAY_CHECK_Pin GPIO_PIN_12
#define MOTOR_RELAY_CHECK_GPIO_Port GPIOC
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_4
#define LED1_GPIO_Port GPIOB
#define LED0_Pin GPIO_PIN_5
#define LED0_GPIO_Port GPIOB
#define EX_CHAG_RLY_CHECK_Pin GPIO_PIN_6
#define EX_CHAG_RLY_CHECK_GPIO_Port GPIOB
#define SMPS_RELAY_CHECK_Pin GPIO_PIN_7
#define SMPS_RELAY_CHECK_GPIO_Port GPIOB
#define RS485_DE_Pin GPIO_PIN_8
#define RS485_DE_GPIO_Port GPIOB
#define RS485_nRE_Pin GPIO_PIN_9
#define RS485_nRE_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
