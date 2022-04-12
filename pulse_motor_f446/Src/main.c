/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lv_fw_common_utility.h"
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
/* USER CODE BEGIN PFP */

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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//#define UNIPOL1
//#define BIPOL1
#define BIPOL2

#ifdef UNIPOL1
#define UNI_POL
#define PULSE_CNT      300  // 256=45度、300=54度、512=90度、768=135度、1024=180度
#define DELTA_FPS      2
#define MIN_FPS        500
#define MAX_FPS        800
#endif

#ifdef BIPOL1
#undef UNI_POL
#define PULSE_CNT      315  // 315=180度
#define DELTA_FPS      5
#define MIN_FPS        500
#define MAX_FPS        1200
#endif

#ifdef BIPOL2
#undef UNI_POL
#define PULSE_CNT      500  // 500=180度
#define DELTA_FPS      5
#define MIN_FPS        500
#define MAX_FPS        1200
#endif
  #define MAX_FPS_OFFSET (MAX_FPS-MIN_FPS)
  uint32_t counter = 0;
  uint32_t fps_offset = 0;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET) {
      lv_fw_common_sync_fps(MIN_FPS);
    } else {
      lv_fw_common_sync_fps(MIN_FPS + fps_offset);
    }
//    HAL_Delay(1);  // 1で2ms=500Hz, 2で3ms=333Hz 周期

    uint16_t phase = counter % PULSE_CNT;
    uint8_t up_down = (counter / PULSE_CNT) % 2;

#ifdef UNI_POL // Uni-Poler Stepping Motor
    switch ((up_down * 4) + (phase & 0x03)) {
      case 0:
      case 7:
        HAL_GPIO_WritePin(D10_GPIO_Port, D10_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(D11_GPIO_Port, D11_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(D12_GPIO_Port, D12_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(D13_GPIO_Port, D13_Pin, GPIO_PIN_RESET);
        break;
      case 1:
      case 6:
        HAL_GPIO_WritePin(D10_GPIO_Port, D10_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(D11_GPIO_Port, D11_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(D12_GPIO_Port, D12_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(D13_GPIO_Port, D13_Pin, GPIO_PIN_RESET);
        break;
      case 2:
      case 5:
        HAL_GPIO_WritePin(D10_GPIO_Port, D10_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(D11_GPIO_Port, D11_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(D12_GPIO_Port, D12_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(D13_GPIO_Port, D13_Pin, GPIO_PIN_SET);
        break;
      case 3:
      case 4:
        HAL_GPIO_WritePin(D10_GPIO_Port, D10_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(D11_GPIO_Port, D11_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(D12_GPIO_Port, D12_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(D13_GPIO_Port, D13_Pin, GPIO_PIN_SET);
        break;
    }
#else // Bi-Poler Stepping Motor
    switch ((up_down * 4) + (phase & 0x03)) {
      case 0:
      case 7:
        HAL_GPIO_WritePin(MTR_A_IA_GPIO_Port, MTR_A_IA_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MTR_A_IB_GPIO_Port, MTR_A_IB_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MTR_B_IA_GPIO_Port, MTR_B_IA_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MTR_B_IB_GPIO_Port, MTR_B_IB_Pin, GPIO_PIN_RESET);
        break;
      case 1:
      case 6:
        HAL_GPIO_WritePin(MTR_A_IA_GPIO_Port, MTR_A_IA_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MTR_A_IB_GPIO_Port, MTR_A_IB_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MTR_B_IA_GPIO_Port, MTR_B_IA_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MTR_B_IB_GPIO_Port, MTR_B_IB_Pin, GPIO_PIN_RESET);
        break;
      case 2:
      case 5:
        HAL_GPIO_WritePin(MTR_A_IA_GPIO_Port, MTR_A_IA_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MTR_A_IB_GPIO_Port, MTR_A_IB_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MTR_B_IA_GPIO_Port, MTR_B_IA_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MTR_B_IB_GPIO_Port, MTR_B_IB_Pin, GPIO_PIN_SET);
        break;
      case 3:
      case 4:
        HAL_GPIO_WritePin(MTR_A_IA_GPIO_Port, MTR_A_IA_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MTR_A_IB_GPIO_Port, MTR_A_IB_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MTR_B_IA_GPIO_Port, MTR_B_IA_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MTR_B_IB_GPIO_Port, MTR_B_IB_Pin, GPIO_PIN_SET);
        break;
    }
#endif
    if(phase >= PULSE_CNT/2) {
      phase = PULSE_CNT - phase - 1;    // 残パルス数
    }
    fps_offset = ((phase * DELTA_FPS) < MAX_FPS_OFFSET) ? (phase * DELTA_FPS) : MAX_FPS_OFFSET;
    counter ++;
  }
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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
