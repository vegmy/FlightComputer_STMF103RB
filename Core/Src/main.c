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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "stdint.h"
#include "stdlib.h"
#include "string.h"
#include "stdio.h"
#include "stts751_temperature_sensor.h"
#include "lsm6dso_imu.h"

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
I2C_HandleTypeDef hi2c1;

/* Definitions for DataPresent */
osThreadId_t DataPresentHandle;
const osThreadAttr_t DataPresent_attributes = {
  .name = "DataPresent",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for DataCollection */
osThreadId_t DataCollectionHandle;
const osThreadAttr_t DataCollection_attributes = {
  .name = "DataCollection",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for DataProcessing */
osThreadId_t DataProcessingHandle;
const osThreadAttr_t DataProcessing_attributes = {
  .name = "DataProcessing",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* Flight computer RTOS shared recourses */
float temperature = 0.0f;
float imu_vector[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
uint8_t lsm6dso_imu_ok = 0;
UART_HandleTypeDef huart2;
int temp_int  = 0;
int gyroX_int = 0;
int gyroY_int = 0;
int gyroZ_int = 0;
int accX_int  = 0;
int accY_int  = 0;
int accZ_int  = 0;
int acceleration_int =0;
int speed_int = 0;

float prev_acceleration = 0.0f;
float acceleration = 0.0f;
float prev_speed   = 0.0f;
float speed        = 0.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
void data_presentation_thread(void *argument);
void data_collection_thread(void *argument);
void data_processing_thread(void *argument);

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
  MX_I2C1_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  char resetbuffer[64];
  HAL_UART_Transmit(&huart2, (uint8_t *)resetbuffer, strlen(resetbuffer), 50);
  sprintf(resetbuffer, "Reset\n");
  lsm6dso_imu_ok = lsm6dso_imu_init(&hi2c1);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of DataPresent */
  DataPresentHandle = osThreadNew(data_presentation_thread, NULL, &DataPresent_attributes);

  /* creation of DataCollection */
  DataCollectionHandle = osThreadNew(data_collection_thread, NULL, &DataCollection_attributes);

  /* creation of DataProcessing */
  DataProcessingHandle = osThreadNew(data_processing_thread, NULL, &DataProcessing_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
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
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_data_presentation_thread */
/**
 * @brief  This thread presents the data to the user by printing to COM.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_data_presentation_thread */
void data_presentation_thread(void *argument)
{
  /* USER CODE BEGIN 5 */
  char buffer[176];
  /* Infinite loop */
  for (;;)
  {
    if (1 != lsm6dso_imu_ok)
    {
      sprintf(buffer, "LSM6DSO init error!");
      HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), 1000);
      osDelay(1000);
    }
    else
    {
      /* Print data to COM */
sprintf(buffer, "Temperature: %d.%02d C, AccX: %d.%02d g, AccY: %d.%02d g, AccZ: %d.%02d g, GyroX: %d.%02d dps, GyroY: %d.%02d dps, GyroZ: %d.%02d dps, Acc: %d.%02d m/s^2, Speed: %d.%02d m/s\r\n",
        temp_int / 100, temp_int % 100,
        accX_int / 100, accX_int % 100,
        accY_int / 100, accY_int % 100,
        accZ_int / 100, accZ_int % 100,
        gyroX_int / 100, gyroX_int % 100,
        gyroY_int / 100, gyroY_int % 100,
        gyroZ_int / 100, gyroZ_int % 100,
        acceleration_int / 100, acceleration_int % 100, // Assuming acceleration is already in m/s^2
        speed_int / 100, speed_int % 100 ); // Assuming speed is calculated correctly in m/s

HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), 1000);
osDelay(100);
    }
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_data_collection_thread */
/**
* @brief Function implementing the DataCollection thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_data_collection_thread */
void data_collection_thread(void *argument)
{
  /* USER CODE BEGIN data_collection_thread */
  char buffer[64];
  uint16_t collection_counter = 0;
  /* Infinite loop */
for (;;)
  {
    stts751_read_temperature(&hi2c1, &temperature);
    lsm6dso_read_imu(&hi2c1, imu_vector);
    osDelay(100);
  }
  /* USER CODE END data_collection_thread */
}
/* USER CODE BEGIN Header_data_processing_thread */
/**
* @brief Function implementing the DataProcessing thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_data_processing_thread */
void data_processing_thread(void *argument)
{
  /* USER CODE BEGIN data_processing_thread */
  char buffer[32];
  uint32_t lastTick = 0;
  uint32_t currentTick;
  float deltaTime;
  /* Infinite loop */
  for(;;)
  {
    prev_acceleration = acceleration;
    temp_int  = (int)(temperature * 100);
    gyroX_int = (int)(imu_vector[0] * 100);
    gyroY_int = (int)(imu_vector[1] * 100);
    gyroZ_int = (int)(imu_vector[2] * 100);
    accX_int  = (int)(imu_vector[3] * 100);
    accY_int  = (int)(imu_vector[4] * 100);
    accZ_int  = (int)(imu_vector[5] * 100);
    acceleration_int = (int)acceleration;
    speed_int = (int)speed;

    acceleration = sqrtf((imu_vector[5] * imu_vector[5]) + (imu_vector[4] * imu_vector[4]) + (imu_vector[3] + imu_vector[3])) * 9.81f;

    currentTick = HAL_GetTick(); // For STM32 HAL, gets the tick since the program started
    deltaTime = (currentTick - lastTick) / 1000.0f; // Convert milliseconds to seconds
    lastTick = currentTick; // Update lastTick for the next measurement

    speed = (acceleration - prev_acceleration) / deltaTime;
    
    osDelay(100);
  }
  /* USER CODE END data_processing_thread */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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


/* SCHEDULER TESTING 
   PRINT TO COM

  /* USER CODE BEGIN
  char buffer[32];
  uint16_t processing_counter = 0;
  /* Infinite loop 
  for(;;)
  {
    sprintf(buffer, "3. Processing: %d \n", processing_counter);
    HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), 1000);
    processing_counter++;
    osDelay(1000);
  }
*/