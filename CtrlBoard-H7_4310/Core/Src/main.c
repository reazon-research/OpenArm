/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dcmi.h"
#include "fdcan.h"
#include "i2c.h"
#include "octospi.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can_bsp.h"
#include "dm_drv.h"
#include "arm_math.h"
#include <stdio.h>
#include "EventRecorder.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CONTROL_PERIOD_US 1000
#define TOGGLE_PERIOD_US 5000000  // 5 seconds in microseconds
OpenArm_t arm;
int received;
extern float vel_set;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern UART_HandleTypeDef huart2;  // Define the UART handle for USART2
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int __io_putchar(int ch)
{
	HAL_UART_Transmit(&huart2,(uint8_t*)&ch,1,HAL_MAX_DELAY);
	return ch;
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
	
//01 08 00 00 02 00 00 00
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
  MX_DCMI_Init();
  MX_FDCAN1_Init();
  MX_FDCAN3_Init();
  MX_I2C1_Init();
  MX_OCTOSPI1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_TIM1_Init();
  MX_UART7_Init();
  MX_UART9_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART10_UART_Init();
  MX_USB_OTG_HS_PCD_Init();
  MX_FDCAN2_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	Power_OUT1_ON;
	Power_OUT2_ON;
	HAL_Delay(200);
	FDCAN1_Config();
	FDCAN2_Config();
	
	int id[NUM_MOTORS];
	int master_id[NUM_MOTORS];
	float zero[NUM_MOTORS];
	float one[NUM_MOTORS];
	float positions[NUM_MOTORS];
	int mode = MIT_MODE;

	int type[8] = {DM4310, DM4310, DM4310, DM4310, DM4310, DM4310, DM4310, DM4310};
	float feedforward_torque = 0.0f;
	
	for (int i = 0; i < NUM_MOTORS; ++i) {
		id[i] = i + 1; // 0x01 to 0x08
		master_id[i] = 0x10 + (i + 1); // 0x11 to 0x18
		type[i] = DM4310;
		zero[i] = 0.0f;
		one[i] = 1.0f;

		positions[i] = 0.0f;
	}

	
	float torque_increment = 0.0001f;
	openarm_init(&arm, id, master_id, mode, type);
	HAL_Delay(1000);
	
	openarm_enable(&arm, &hfdcan1);
	EventRecorderInitialize(EventRecordAll, 1);
	HAL_TIM_Base_Start(&htim2);
	
	
	uint32_t toggle_timer = 0;
	uint8_t toggle = 0;
	uint32_t last_time = 0;
	uint32_t now = 0;
	uint32_t t_schedule = 0;
	__HAL_TIM_SET_COUNTER(&htim2, 0);  // Reset timer to avoid drift
	
	//change_baudrate(&hfdcan1, 1, BAUD_1M);
	
  while (1)
  { 
		
		uint32_t now = __HAL_TIM_GET_COUNTER(&htim2);
    uint32_t elapsed = now - last_time;

    t_schedule += CONTROL_PERIOD_US;

    int32_t td = t_schedule - __HAL_TIM_GET_COUNTER(&htim2);

    if (td > 0) {
				td += 0xFFFFFFFF; // Account for overflow
        while (__HAL_TIM_GET_COUNTER(&htim2) < t_schedule);  // Wait for the required time
        //printf("Control loop took %d us. Waiting for: %d us\n\r", elapsed, td);
				
    } else {
        printf("WARNING: Control loop overran by %d us!\n", -td);
    }
    toggle_timer += CONTROL_PERIOD_US;

    if (toggle_timer >= TOGGLE_PERIOD_US) {
        toggle ^= 1;
        toggle_timer = 0;
    }
				
		last_time = __HAL_TIM_GET_COUNTER(&htim2);
		EventRecord2(0x03, positions[0]*100, 0x01);

//		if(positions[0] < 1.0f){
//			for (int i = 0; i < NUM_MOTORS; i++) {
//				positions[i] += torque_increment;
//			}
//			move_mit_all(&arm, &hfdcan1, zero, zero, zero, zero, positions);
//		}
//		else{
//			if (toggle) {
//					move_mit_all(&arm, &hfdcan1, zero, zero, zero, zero, one);
//			} else {
//					move_mit_all(&arm, &hfdcan1, zero, zero, zero, zero, zero);
//			}
//		}
//		mit_ctrl(&hfdcan1, 1, 0.0, 0.0, 0.0, 0.0, 0.0);
		if (toggle) {
				move_mit_all(&arm, &hfdcan1, zero, zero, zero, zero, one);
		} else {
				move_mit_all(&arm, &hfdcan1, zero, zero, zero, zero, zero);
		}
		
		//    printf("TIM2 Counter: %u\n", __HAL_TIM_GET_COUNTER(&htim2));
		//    HAL_Delay(500); // Print every 500ms
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
	
	openarm_disable(&arm, &hfdcan1);
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 3;
  RCC_OscInitStruct.PLL.PLLN = 62;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 4096;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSE, RCC_MCODIV_1);
}

/* USER CODE BEGIN 4 */

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
