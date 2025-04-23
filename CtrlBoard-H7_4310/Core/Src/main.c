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
#include "dma.h"
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
#include "key_bsp.h"
#include "led_bsp.h"
#include <stdbool.h>
#include "control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CONTROL_PERIOD_US 1000
#define TOGGLE_PERIOD_US 5000000  // 5 seconds in microseconds
#define HDR1 0xAA
#define HDR2 0x55
#define ARRAY_SIZE 14

OpenArm_t arm;
OpenArm_t arm2;
BilateralInfo_t leader_info;
BilateralInfo_t follower_info;


int received;
extern float vel_set;
#define UART_RX_BUFFER_SIZE 87
uint8_t rx_buffer[UART_RX_BUFFER_SIZE];
// uint8_t tx_buffer[59];  // ヘッダ（2バイト）+ 位置データ（28バイト）+ 速度データ（28バイト）+ CRC（1バイト）

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
void send_data(UART_HandleTypeDef *huart, uint8_t* data, size_t len)
{
    // DMA送信を開始
    HAL_UART_Transmit_DMA(huart, data, len);
}

uint8_t calcCRC(uint8_t* data, size_t len)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
    }
    return crc;
}

void sendFloatArray(UART_HandleTypeDef *huart, float arr[ARRAY_SIZE]) {
  uint8_t crc = 0;
  uint8_t header[2] = { HDR1, HDR2 };

  // 位置データとCRCを含む全データを格納するバッファを準備
  uint8_t tx_buffer[ARRAY_SIZE * sizeof(float) + sizeof(header) + 1];  // ヘッダ + 位置データ + CRC

  // ヘッダをバッファに格納
  memcpy(tx_buffer, header, sizeof(header));

  // 位置データをバッファに格納
  const uint8_t* arr_ptr = (const uint8_t*)arr;  // float 配列をバイト配列として扱う
  for (size_t i = 0; i < ARRAY_SIZE * sizeof(float); ++i) {
      tx_buffer[sizeof(header) + i] = arr_ptr[i];  // ヘッダの後に位置データを格納
  }

  // CRC計算（データ部分のみ）
  for (size_t i = 2; i < sizeof(tx_buffer) - 1; ++i) {  // ヘッダを除いてCRCを計算
      crc ^= tx_buffer[i];
  }

  // CRCをバッファの最後に格納
  tx_buffer[sizeof(tx_buffer) - 1] = crc;

  // ヘッダ + 位置データ + CRC を一度に送信
  HAL_UART_Transmit_DMA(huart, tx_buffer, sizeof(tx_buffer));  // DMAで全データを一度に送信
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{

        huart1.gState = HAL_UART_STATE_READY;
        
}

float m_diag[7];    // M_diagのデータを格納する配列
float coriolis[7];  // Coriolisのデータを格納する配列
float gravity[7];   // Gravityのデータを格納する配列
uint8_t crc;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        static uint8_t data_stage = 0;  // 受信段階を管理するフラグ

        // 受信データの処理
        switch (data_stage) {
            case 0:  // ヘッダ受信
                if (rx_buffer[0] == 0xAA && rx_buffer[1] == 0x55) {
                    // ヘッダが正しい場合
                    //printf("Received header: 0xAA 0x55\n");
                    data_stage = 1;  // 次は M_diag のデータを受信
                } else {
                    //printf("Header error!\n");
                }
                break;

            case 1:  // M_diag 受信
                // 28バイトのM_diagデータを処理
                for (int i = 0; i < 7; i++) {
                    m_diag[i] = *((float*)&rx_buffer[i * 4]);
                    // printf("M_diag[%d]: %f\n", i, m_diag[i]);
                }
                data_stage = 2;  // 次は Coriolis のデータを受信
                break;

            case 2:  // Coriolis 受信
                // 28バイトのCoriolisデータを処理
                for (int i = 0; i < 7; i++) {
                    coriolis[i] = *((float*)&rx_buffer[i * 4]);
                    // printf("Coriolis[%d]: %f\n", i, coriolis[i]);
                }
                data_stage = 3;  // 次は Gravity のデータを受信
                break;

            case 3:  // Gravity 受信
                // 28バイトのGravityデータを処理
                for (int i = 0; i < 7; i++) {
                    gravity[i] = *((float*)&rx_buffer[i * 4]);
                    // printf("Gravity[%d]: %f\n", i, gravity[i]);
                }
                data_stage = 4;  // 次は CRC を受信
                break;

            case 4:  // CRC 受信
                crc = rx_buffer[0];  // CRCの受信処理
                // printf("Received CRC: 0x%02X\n", crc);
                // CRCの検証処理を追加
                data_stage = 0;  // 次のデータを受信する準備
                break;

            default:
                data_stage = 0;
                break;
        }

        // 受信後に再度DMAによる受信を開始
        HAL_UART_Receive_DMA(&huart1, rx_buffer, sizeof(rx_buffer));
    }
}

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
  MX_DMA_Init();
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

  float tau[NUM_MOTORS];
  float tau2[NUM_MOTORS];
  float vel[NUM_MOTORS];
  float vel2[NUM_MOTORS];
  float pos[NUM_MOTORS];
  float pos2[NUM_MOTORS];

	
	int typel[NUM_MOTORS] = {DM4340, DM4340, DM4340, DM4340, DM4310, DM4310, DM4310, DM4340};
	int typef[NUM_MOTORS] = {DM4340, DM4340, DM4340, DM4340, DM4310, DM4310, DM4310, DM4340};

	for (int i = 0; i < NUM_MOTORS; ++i) {
		id[i] = i + 1; // 0x01 to 0x08
		master_id[i] = 0x10 + (i + 1); // 0x11 to 0x18
		//type[i] = DM8009;
		zero[i] = 0.0f;
		one[i] = 1.0f;
		positions[i] = 0.0f;
	}

  //openarm setting 
	openarm_init(&arm, id, master_id, mode, typel);
	openarm_init(&arm2, id, master_id, mode, typef);
	HAL_Delay(1000);


  //bilateral control setting 
  float Ts = 0.001f;

  float Dn_leader[NUM_MOTORS] =  {0.1f, 0.1f, 0.03f, 0.03f, 0.03f, 0.003f, 0.003f, 0.001f};
  float Jn_leader[NUM_MOTORS] =  {0.03f, 0.0007f, 0.0007f, 0.0007f, 0.0007f, 0.0007f, 0.0007f, 0.0007f};
  float gnd_leader[NUM_MOTORS] = {10.0f, 10.0f, 10.0f, 10.0f, 5.0f, 5.0f, 5.0f, 0.0f};
  float gnf_leader[NUM_MOTORS] = {10.0f, 10.0f, 10.0f, 10.0f, 5.0f, 5.0f, 5.0f, 0.0f};
  float Gn_leader[NUM_MOTORS] =  {3.0f, 2.0f, 0.0f, 0.0f, 0.0f, 0.4f, 0.0f, 0.0f};
  float Kp_leader[NUM_MOTORS] =  {160.0f, 140.0f, 110.0f, 110.0f, 12.0f, 15.0f, 15.0f, 10.0f};
  float Kd_leader[NUM_MOTORS] =  {3.4f, 2.4f, 1.4f, 1.4f, 0.4f, 0.4f, 0.4f, 0.05f};
  float Kf_leader[NUM_MOTORS] =  {0.7f, 0.7f, 0.7f, 0.7f, 0.4f, 0.4f, 0.4f, 0.0f};

  float Dn_follower[NUM_MOTORS] =  {0.1f, 0.1f, 0.03f, 0.03f, 0.03f, 0.003f, 0.003f, 0.001f};
  float Jn_follower[NUM_MOTORS] =  {0.03f, 0.0007f, 0.0007f, 0.0007f, 0.0007f, 0.0007f, 0.0007f, 0.000f};
  float gnd_follower[NUM_MOTORS] = {10.0f, 10.0f, 10.0f, 10.0f, 5.0f, 5.0f, 5.0f, 0.0f};
  float gnf_follower[NUM_MOTORS] = {10.0f, 10.0f, 10.0f, 10.0f, 5.0f, 5.0f, 5.0f, 0.0f};
  float Gn_follower[NUM_MOTORS] =  {3.0f, 2.0f, 0.0f, 0.0f, 0.0f, 0.4f, 0.0f, 0.0f};
  float Kp_follower[NUM_MOTORS] =  {160.0f, 140.0f, 110.0f, 110.0f, 12.0f, 15.0f, 15.0f, 10.0f / GRIP_SCALE};
  float Kd_follower[NUM_MOTORS] =  {3.4f, 2.4f, 1.4f, 1.4f, 1.4f, 0.4f, 0.4f, 0.05f / GRIP_SCALE};
  float Kf_follower[NUM_MOTORS] =  {0.7f, 0.7f, 0.7f, 0.7f, 0.4f, 0.4f, 0.4f, 0.0f};

  float disturbance_leader[NUM_MOTORS] = {0};
  float reactionforce_leader[NUM_MOTORS] = {0};
  float disturbance_in_leader[NUM_MOTORS] = {0};
  float disturbance_out_leader[NUM_MOTORS] = {0};
  float reaction_in_leader[NUM_MOTORS] = {0};
  float reaction_out_leader[NUM_MOTORS] = {0};
  float joint_torque_leader[NUM_MOTORS] = {0};

  float disturbance_follower[NUM_MOTORS] = {0};
  float reactionforce_follower[NUM_MOTORS] = {0};
  float disturbance_in_follower[NUM_MOTORS] = {0};
  float disturbance_out_follower[NUM_MOTORS] = {0};
  float reaction_in_follower[NUM_MOTORS] = {0};
  float reaction_out_follower[NUM_MOTORS] = {0};
  float joint_torque_follower[NUM_MOTORS] = {0};

  init_bilateral_info(&leader_info, Ts, ROLE_LEADER, &arm, &hfdcan1,
                      Dn_leader, Jn_leader, gnd_leader, gnf_leader,
                      Gn_leader, Kp_leader, Kd_leader, Kf_leader,
                      disturbance_in_leader, disturbance_out_leader,
                      disturbance_leader, reaction_in_leader,
                      reaction_out_leader, reactionforce_leader ,joint_torque_leader);

  init_bilateral_info(&follower_info, Ts, ROLE_FOLLWER, &arm2, &hfdcan2,
                      Dn_follower, Jn_follower, gnd_follower, gnf_follower,
                      Gn_follower, Kp_follower, Kd_follower, Kf_follower,
                      disturbance_in_follower, disturbance_out_follower,
                      disturbance_follower, reaction_in_follower,
                      reaction_out_follower, reactionforce_follower ,joint_torque_follower);

  // // OpenArm baudrate change(hfdcan1)
  // for (uint8_t id = 0x01; id <= 0x08; ++id) {
  //     write_baudrate(&hfdcan1, id, BAUD_1M);  // 1Mのボーレートを設定
  //     HAL_Delay(500);  // 500msの遅延
  // }


	// openarm_enable(&arm, &hfdcan1);
	// openarm_enable(&arm2, &hfdcan2);
	
	printf("EVENT RECORDER BEFORE\n\r");
	//EventRecorderInitialize(EventRecordAll, 1);
	HAL_TIM_Base_Start(&htim2);

  // openarm_set_zero_position(&arm, &hfdcan1);
  // openarm_set_zero_position(&arm2, &hfdcan2);
	
	uint32_t toggle_timer = 0;
	uint8_t toggle = 0;
	uint32_t last_time = 0;
	uint32_t now = 0;
	uint32_t t_schedule = 0;
	__HAL_TIM_SET_COUNTER(&htim2, 0);  // Reset timer to avoid drift

  GPIO_PinState last_key_state = GPIO_PIN_SET;
  bool torque_disabled = false;

  // HAL_UART_Receive_IT(&huart1, rx_buffer, sizeof(rx_buffer));
  // HAL_UART_Receive_DMA(&huart1, rx_buffer, sizeof(rx_buffer));

	// HAL_UART_Transmit_DMA(&huart1, (uint8_t*) "Boot NUCLEO\r\n", 13);
	// while (huart1.gState != HAL_UART_STATE_READY) {
	// }

  float tx_test[ARRAY_SIZE] = {4.0f, 4.0f, 4.0f, 4.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};


  sendFloatArray(&huart1, tx_test);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
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
		// EventRecord2(0x03, positions[0]*100, 0x01);

    // bilateral control
    // bool use_bilate = true;
    // if(use_bilate){
    //   bilateral_control_v1(&leader_info, &follower_info);
    // }
    // else {
    // //  move_mit_all(&arm, &hfdcan1, zero, zero, leader_info.Kp, leader_info.Kd, zero);
    // //  move_mit_all(&arm2, &hfdcan2, zero, zero, follower_info.Kp, follower_info.Kd, zero);
    // move_mit_all(&arm, &hfdcan1, zero, zero, zero, zero, zero);
    // move_mit_all(&arm2, &hfdcan2, zero, zero, zero, zero, zero);
    // }

    //  move_mit_all(&arm, &hfdcan1, zero, zero, leader_info.Kp, leader_info.Kd, zero);
    //  move_mit_all(&arm2, &hfdcan2, zero, zero, follower_info.Kp, follower_info.Kd, zero);


		// if (toggle) {
		// 		move_mit_all(&arm, &hfdcan1, zero, zero, zero, zero, zero);
		// 		move_mit_all(&arm2, &hfdcan2, zero, zero, zero, zero, zero);
		// } else {
		// 		move_mit_all(&arm, &hfdcan1, zero, zero, zero, zero, zero);
    //   	move_mit_all(&arm2, &hfdcan2, zero, zero, zero, zero, zero);
		// }

    if (key_1 == GPIO_PIN_RESET) {
        printf("pushed\n\r");
    }

    printf("pos 1 : %f pos 2 : %f\n\r", arm2.motors[0].pos, arm2.motors[1].pos);
	  // printf("dis : %f \n\r", leader_info.disturbance[0]);
	  // printf("dis : %f \n\r", arm.motors[1].vel);

    // if (!torque_disabled) {
    //     GPIO_PinState current_key_state = key_bsp_read_pin(GPIOA, GPIO_PIN_15);

    //     if (last_key_state == GPIO_PIN_SET && current_key_state == GPIO_PIN_RESET) {
    //         HAL_Delay(20);
    //         if (key_bsp_read_pin(GPIOA, GPIO_PIN_15) == GPIO_PIN_RESET) {
    //             printf("pushed - disabling torque!\n\r");
    //             openarm_disable(&arm, &hfdcan1);
    //             openarm_disable(&arm2, &hfdcan2);
    //             torque_disabled = true;
    //         }
    //     }

    //     last_key_state = current_key_state;
    // }

      // 現在のボタンの状態を取得
      GPIO_PinState current_key_state = key_bsp_read_pin(GPIOA, GPIO_PIN_15);

      // ボタンが押された瞬間を検出
      if (last_key_state == GPIO_PIN_SET && current_key_state == GPIO_PIN_RESET) {
          HAL_Delay(20);  // チャタリング防止のため、少し待つ

          // 再度ボタンが押されたことを確認
          if (key_bsp_read_pin(GPIOA, GPIO_PIN_15) == GPIO_PIN_RESET) {
              // ボタンが押されたとき、sendFloatArrayを呼び出して送信
              sendFloatArray(&huart1, tx_test);
          }
      }

      // ボタンの状態を次回のチェックのために保存
      last_key_state = current_key_state;

		
		// printf("TIM2 Counter: %u\n", __HAL_TIM_GET_COUNTER(&htim2));
		// HAL_Delay(500); // Print every 500ms
    // HAL_Delay(1);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
	
	// openarm_disable(&arm, &hfdcan1);
	// openarm_disable(&arm2, &hfdcan2);
	
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
