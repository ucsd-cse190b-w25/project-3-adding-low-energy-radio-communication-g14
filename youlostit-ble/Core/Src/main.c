/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
//#include "ble_commands.h"
#include "ble.h"
#include "leds.h"
// #include "timer.h"
#include <stm32l475xx.h>
#include "lsm6dsl.h"
#include "lptimer.h"
#include <stdio.h>
#include <stdlib.h>

int dataAvailable = 0;

SPI_HandleTypeDef hspi3;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI3_Init(void);

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

volatile uint32_t updated_count = 0;
volatile uint32_t count_ten = 0;
volatile uint32_t time = 0;
volatile uint32_t discover = 0;

/**
 * @brief  LPTIM1 Interrupt Handler
 * @note   Triggered on LPTIM1 auto-reload match (periodic timer)
 */
void LPTIM1_IRQHandler(void)
{
	if (LPTIM1->ISR & LPTIM_ISR_ARRM) {
		LPTIM1->ICR |= LPTIM_ICR_ARRMCF; // Clear the interrupt flag
		updated_count++;	// Increment time counter
	}

};

/**
 * @brief  Main program entry point
 * @retval int
 */
int main(void)
{
  /* MCU Configuration and Initialization */
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_SPI3_Init();

  /* BLE Module Reset Sequence */
  HAL_GPIO_WritePin(BLE_RESET_GPIO_Port, BLE_RESET_Pin, GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(BLE_RESET_GPIO_Port, BLE_RESET_Pin, GPIO_PIN_SET);

  /* Initialize Peripherals */
  ble_init();
  HAL_Delay(10);

  /* Initialize LPTIM1 as 5 second periodic timer */
  lptimer_init(LPTIM1);
  lptimer_set_ms(LPTIM1, 1250);

  /* Initialize I2C and LSM6DSL accelerometer */
  i2c_init();
  for (volatile int i = 0; i < 500000; i++);  // Delay for stabilization
  lsm6dsl_init();

  /* Accelerometer variables */
  int16_t x, y, z;                // Current accelerometer readings
  int16_t lastX, lastY, lastZ;    // Previous accelerometer readings
  const int16_t thresh = 45;      // Movement detection threshold

  /* State variables */
  int is_moved_bc = 0;            // Flag for BLE message broadcast
  uint8_t nonDiscoverable = 0;    // BLE discoverability control
  setDiscoverability(1);          // Start in discoverable mode

  /* Main loop */
  while (1)
  {
      /* Check for BLE interrupt if in discoverable mode */
      if (!nonDiscoverable && HAL_GPIO_ReadPin(BLE_INT_GPIO_Port, BLE_INT_Pin)) {
          catchBLE();  // Handle BLE interrupt
      }

      /* Active Mode (< 60 seconds since last movement) */
      if (updated_count < 12) {    // Less than 60 seconds (12 * 5s)
          setDiscoverability(0);   // Set to non-discoverable to save power

          /* Read accelerometer data */
          lsm6dsl_read_xyz(&x, &y, &z);

          /* Check for significant movement in any axis */
          if (x < lastX - thresh || x > lastX + thresh) {
              lastX = x;
              updated_count = 0;  // Reset timer on movement
              is_moved_bc = 0;
              time = 0;
          }
          if (y < lastY - thresh || y > lastY + thresh) {
              lastY = y;
              updated_count = 0;
              is_moved_bc = 0;
              time = 0;
          }
          if (z < lastZ - thresh || z > lastZ + thresh) {
              lastZ = z;
              updated_count = 0;
              is_moved_bc = 0;
              time = 0;
          }
          is_moved_bc = 0;  // Reset broadcast flag
      }
      /* Lost Mode (≥ 60 seconds without movement) */
      else {
          setDiscoverability(1);  // Make device discoverable

          /* Handle counter overflow */
          if (updated_count == 0xFFFFFFFF) {
              updated_count = 14;  // Reset to slightly above threshold
          }

          /* Send initial lost notification */
          if (is_moved_bc == 0) {
              count_ten = updated_count;
              unsigned char test_str[] = "Airtag1 lost 0s";
              HAL_Delay(1000);
              updateCharValue(NORDIC_UART_SERVICE_HANDLE, READ_CHAR_HANDLE, 0, sizeof(test_str)-1, test_str);
              is_moved_bc = 1;  // Set broadcast flag to prevent repeated initial messages
          }

          /* Send periodic updates every 2.5 seconds */
          if (count_ten == updated_count - 2) {
              time += 10;  // Increment lost time counter
              count_ten = updated_count;  // Update reference point

              /* Format and send lost status update */
              char str[100];
              snprintf(str, sizeof(str), "Airtag1 lost %ds", time);
              HAL_Delay(1000);
              updateCharValue(NORDIC_UART_SERVICE_HANDLE, READ_CHAR_HANDLE, 0, strlen(str), (unsigned char*)str);
          }

          /* Continue checking for movement to exit lost mode */
          lsm6dsl_read_xyz(&x, &y, &z);

          /* Check for significant movement in any axis */
          if (x < lastX - thresh || x > lastX + thresh) {
              lastX = x;
              updated_count = 0;  // Reset counters
              time = 0;
              disconnectBLE();    // Disconnect BLE when found
          }
          if (y < lastY - thresh || y > lastY + thresh) {
              lastY = y;
              updated_count = 0;
              time = 0;
              disconnectBLE();
          }
          if (z < lastZ - thresh || z > lastZ + thresh) {
              lastZ = z;
              updated_count = 0;
              time = 0;
              disconnectBLE();
          }
      }

      /* Power saving: Suspend BLE if non-discoverable */
      if (nonDiscoverable) {
          standbyBle();
      }

      /* Enter low-power STOP2 mode between operations */
      SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;  // Configure for normal sleep

      /* Suspend SysTick to save power */
      HAL_SuspendTick();
      SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;

      /* Enter STOP2 mode, wake on interrupt */
      HAL_PWREx_EnterSTOP2Mode(PWR_SLEEPENTRY_WFI);

      /* Resume SysTick after wakeup */
      SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
      HAL_ResumeTick();
  }
}

/**
  * @brief System Clock Configuration
  * @attention This changes the System clock frequency, make sure you reflect that change in your timer
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
      Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  // This line changes system clock frequency
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_7;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
      Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
      Error_Handler();
  }
}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{
  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
      Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIO_LED1_GPIO_Port, GPIO_LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BLE_CS_GPIO_Port, BLE_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BLE_RESET_GPIO_Port, BLE_RESET_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : BLE_INT_Pin */
  GPIO_InitStruct.Pin = BLE_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BLE_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_LED1_Pin BLE_RESET_Pin */
  GPIO_InitStruct.Pin = GPIO_LED1_Pin | BLE_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BLE_CS_Pin */
  GPIO_InitStruct.Pin = BLE_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(BLE_CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
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
