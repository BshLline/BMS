/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	return ch;
}

void check_smps_charge(void)
{
	uint8_t	i;
	uint16_t high_voltage = 0, low_voltage = 0xFFFF;

	// Check cell voltage
	for(i=0;i<NUM_OF_CELL;i++)
	{
		// Find highest and lowest voltage cell
		if(Adbms6817.voltage_cell[i] > high_voltage)
		{
			high_voltage = Adbms6817.voltage_cell[i];
		}
		if(Adbms6817.voltage_cell[i] < low_voltage)
		{
			low_voltage = Adbms6817.voltage_cell[i];
		}
	}

	if(low_voltage/10000. < 3.5)
	{
		PWM_Toggle_Relay(PWM_SMPS_CHARGE, OFF);
	}

}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_CAN_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_RTC_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

	HAL_Delay(100);
	HAL_TIM_Base_Start_IT(&htim2);

	HAL_UART_Receive_IT(&huart1, &g_uUart1_Rx_now, 1);
	HAL_UART_Receive_IT(&huart2, &g_uUart2_Rx_now, 1);
	HAL_UART_Receive_IT(&huart3, &g_u485_Rx_now, 1);

	// Initialize ADBMS6817
	//Adbms6817_Set_default();
	Adbms6817_initialize();
	//ADBMS6817 Init Delay
	Adc_dma_start();						//adc infos
	HAL_Delay(1000);

	// Debug Test
	//printf("LLINE BMS 64p Start\r\n");
	Setup_default();
	Setup_ADC_default();
	Setup_uart_default(); 					//Serial buff reset
	Setup_uart485_default();				//485 buff reset
	Get_Switch_info();						//user select switch

	HAL_Delay(100);

	Can_setup_default(Sw_info.g_uOption_id);	//Sw_info.Can_id

	//Battery read	//
	//Battery ok ? ->relay on
	//Battery Low? -> wait? or low battery display
	Start_Relay_pwm(1);
	//Charge_voltage_open(1);
	//Discharge_voltage_open(1);
	Adbms6817_read_adc();
	g_bFlag_5Sec = 1;
#ifdef USE_CELL_BALANCING
	Bms.EnableBalancing = TRUE;
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	printf("L.LINE Battery Management System\r\n\r\n");
	boot_time_hour = boot_time_min = boot_time_sec = 0;
	//for(int i=1;i<42;i++)
		//HAL_RTCEx_BKUPWrite(&hrtc, i, 0x1000+i);

#if 1
	while(1)
	{
		//if(g_uUart1_Rx_Flag) Uart1_Cmd_check();
		Uart1_Cmd_check();
		if(g_uUart2_Rx_Flag) Uart2_Cmd_check();
		if(g_u485_Rx_Flag)	U3_485_Cmd_check();
		if(g_uCan_Rxflag)	Can_Cmd_check();

#if defined(DEBUG_SERIAL) && defined(DEBUG_SERIAL_TEST_BSH)
		Check_Debug_Serial();
#endif
		if(g_bFlag_100ms)
		{
			// �???��?�� 교수 루틴
			EKF_Algorithm_Main();
			Bms.Soc = ed_EKF_SOC*100;
			g_bFlag_100ms = FALSE;
		}
		if(g_bFlag_1Sec)
		{
			//uint8_t data[1024];
			//flash_read(0, data, 1024);
			//HAL_FLASH_Program(TypeProgram, Address, Data);
			//HAL_FLASHEx_Erase(pEraseInit, PageError);

			// BSM Basic Routine
			//Adbms6817_discharge_cell_continuously(0, FALSE);
			//Adbms6817_discharge_cell_continuously(0, TRUE);
			//BMS_Cell_Balancing();
			//Calc_SOC_Voltage();
			//Calc_SOC_Current();
			Check_Relay_Contorl();

			if(Bms.EnableBalancing)
				BMS_Cell_Balancing();
			else
				Adbms6817_read_adc();

			//printf("Bms.Voltage_avg: %d\r\n", Bms.Voltage_avg);
			//Adbms6817_adcv();
			Read_DMA_Current_n_Temperature();
			Broadcast_data();	// Broadcast to uart1
#if defined(DEBUG_SERIAL) && defined(DEBUG_SERIAL_TEST_BSH)
			Print_Cell_Voltage();
			//Print_adc_value();
			//printf("SOC : %f\%, V1 : %f, Vt : %f\r\n\r\n", ed_EKF_SOC*100, ed_EKF_V1, ed_EKF_Vt);
			#if 0
			if(IS_SMPS_RELAY_CHECK)
				printf("SMPS ON\r\n");
			else
				printf("SMPS OFF\r\n");
			if(IS_EX_CHARGING_RELAY_CHECK)
				printf("EX-CHARGE ON\r\n");
			else
				printf("EX-CHARGE OFF\r\n");
			if(IS_MOTOR_RELAY_CHECK)
				printf("MOTOR ON\r\n");
			else
				printf("MOTOR OFF\r\n");
			//printf("txtxtxtx\r\n");
			#endif
#endif
			//if(Err.g_uOver_Current_flag || Err.g_uOver_Charge_flag || Err.g_uOver_Temp_flag) Charge_Discharge_control();

			HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
			g_bFlag_1Sec = 0;
		}

		if(g_bFlag_5Sec)
		{
			//Fan_pwm_control();
			if(Bms.EnableBalancing)
			{
				//BMS_Cell_Balancing();
			}
			g_bFlag_5Sec = 0;
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
#endif

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USB_LP_CAN1_RX0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USART3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
#if 0
	if (huart->Instance == USART1)	//
	{
		g_uUart1_Rx_prev = g_uUart1_Rx_buffer[g_uUart1_Rx_head-1];
		g_uUart1_Rx_buffer[g_uUart1_Rx_head++] =	g_uUart1_Rx_now;

		if(g_uUart1_Rx_now == '\n')
		{
			if(g_uUart1_Rx_prev == '\r')
			{
				uint8_t i=0;
				g_uUart1_Rx_Flag = 1;
				while(g_uUart1_Rx_head != g_uUart1_Rx_tail)
				{
					g_uUart1_Rx_Cmd[i++] = g_uUart1_Rx_buffer[g_uUart1_Rx_tail++];
				}
			}
		}
		HAL_UART_Receive_IT(&huart1, &g_uUart1_Rx_now, 1);
	}
#endif
	if (huart->Instance == USART1)	//
	{
		g_uUart1_Rx_buffer[g_uUart1_Rx_head++] =	g_uUart1_Rx_now;
		HAL_UART_Receive_IT(&huart1, &g_uUart1_Rx_now, 1);
	}
	else if (huart->Instance == USART2)	//
	{
		g_uUart2_Rx_prev = g_uUart2_Rx_buffer[g_uUart2_Rx_head-1];
		g_uUart2_Rx_buffer[g_uUart2_Rx_head++] =	g_uUart2_Rx_now;

#if defined(DEBUG_SERIAL) && defined(DEBUG_SERIAL_TEST_BSH)
		g_uDebug_Rx_buffer[g_uDebug_Rx_head++] =	g_uUart2_Rx_now;
#endif

		if(g_uUart2_Rx_now == '\n')
		{
			if(g_uUart2_Rx_prev == '\r')
			{
				uint8_t i=0;
				g_uUart2_Rx_Flag = 1;
				while(g_uUart2_Rx_head != g_uUart2_Rx_tail)
				{
					g_uUart2_Rx_Cmd[i++] = g_uUart2_Rx_buffer[g_uUart2_Rx_tail++];
				}
			}
		}
		HAL_UART_Receive_IT(&huart2, &g_uUart2_Rx_now, 1);
	}
	else if (huart->Instance == USART3)	//
	{

		g_u485_Rx_prev = g_u485_Rx_buffer[g_u485_Rx_head-1];
		g_u485_Rx_buffer[g_u485_Rx_head++] =g_u485_Rx_now;

		if(g_u485_Rx_now == '\n')
		{
			if(g_u485_Rx_prev == '\r')
			{
				uint8_t i=0;
				g_u485_Rx_Flag = 1;
				while(g_u485_Rx_head != g_u485_Rx_tail)
				{
					g_u485_Rx_Cmd[i++] = g_u485_Rx_buffer[g_u485_Rx_tail++];
				}
			}
		}
		HAL_UART_Receive_IT(&huart3, &g_u485_Rx_now, 1);
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanHandle)
//void CAN1_RX1_IRQHandler(void)
{
	/* Get RX message */
	if (HAL_CAN_GetRxMessage(CanHandle, CAN_RX_FIFO0, &g_Can_RxHeader, g_uCan_RxData) != HAL_OK)
	{
		/* Reception Error */
		Error_Handler();
	}
	g_uCan_Rxflag = 1;
	//printf("StdID: %04lx, IDE: %ld, DLC: %ld\r\n", g_Can_RxHeader.StdId, g_Can_RxHeader.IDE, g_Can_RxHeader.DLC);
	//printf("%04lx Data: %d %d %d %d %d %d %d %d\r\n", g_uCan_RxData[0], g_uCan_RxData[1], g_uCan_RxData[2], g_uCan_RxData[3], g_uCan_RxData[4], g_uCan_RxData[5], g_uCan_RxData[6], g_uCan_RxData[7]);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == htim2.Instance)	//Timer 100ms
	{
		tickCnt_100ms++;
		g_bFlag_100ms = TRUE;
		if(tickCnt_100ms %10 == 0)
		{
			g_bFlag_1Sec = TRUE;
			tickCnt_1s++;
			calc_boot_time();
			if(tickCnt_1s%5 == FALSE)	g_bFlag_5Sec = TRUE;

			Check_bms_error();
		}


	}
}

// CAN Error 콜백
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
	//printf("HAL_CAN_ErrorCallback\r\n");
}

// PVD Callback
void HAL_PWR_PVDCallback()
{

}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
