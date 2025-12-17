/* USER CODE BEGIN Header */
/**
  ****************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ****************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stm32f4xx_hal.h"
#include "liquidcrystal_i2c.h"
#include "dht11.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
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

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//mutex lock
int g_temp = 0;
int g_humi = 0;
//mutex lock
SemaphoreHandle_t xDataMutex;
DHT11_DataTypedef dht11_data;
uint8_t result, Presence;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void DHT11Task(void *pvParameters);
void LCDTask(void *pvParameters);
void UARTTask(void *pvParameters); 
void AlertTask(void *pvParameters);  
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t temp_threshold = 30;
uint8_t humi_threshold = 70;
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
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim6);

  /* 關鍵修正：確保 UART 完全初始化 */
  HAL_Delay(100);

  /* 初始化 LCD（一次） */
  HD44780_Init(2);
  HD44780_Clear();
  HD44780_Backlight();

  char uart_buf[100];
  char lcd_temp[100];
  char lcd_humi[100];
  char lcd_buf[100];
  int uart_buf_len;

  /* 發送啟動訊息 - 測試 UART 是否正常 */
  uart_buf_len = sprintf(uart_buf, "\r\n=== STM32 DHT11 Started ===\r\n");
  HD44780_SetCursor(0,0);
  HD44780_PrintStr("DHT11 Start");
  HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, uart_buf_len, HAL_MAX_DELAY);

  /* 關鍵修正：DHT11 初始化序列 */
  uart_buf_len = sprintf(uart_buf, "Initializing DHT11...\r\n");
  HD44780_SetCursor(0,1);
  HD44780_PrintStr("Initializing");
  HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, uart_buf_len, HAL_MAX_DELAY);

  /* 確保 DHT11 引腳狀態正確 */
  Set_Pin_Output(DHT11_PORT, DHT11_PIN);
  HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, 1);  // 確保高電平
  HAL_Delay(1000);  // 等待 DHT11 穩定

  uart_buf_len = sprintf(uart_buf, "DHT11 initialization complete\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, uart_buf_len, HAL_MAX_DELAY);

  // 建立互斥鎖
  xDataMutex = xSemaphoreCreateMutex();

  // 建 DHT11Task
  xTaskCreate(DHT11Task, "DHT11", 128, NULL, 2, NULL);
  // 建立 LCDTask
  xTaskCreate(LCDTask, "LCD", 128, NULL, 1, NULL);
  // 建立 UARTTask
  xTaskCreate(UARTTask, "UART", 128, NULL, 1, NULL);
  /* 建立紅燈警示 Task */
  xTaskCreate(AlertTask, "Alert", 128, NULL, 2, NULL);

  vTaskStartScheduler();
  while (1) {}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 50-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : RED_LED_Pin */
  GPIO_InitStruct.Pin = RED_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RED_LED_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void DHT11Task(void *pvParameters)
{
    int error_count = 0;
    for (;;)
    {
        // 1. 讀取 DHT11
        result = DHT11_Read_Data(&dht11_data);

        // 2. 如果讀取成功，就更新全域變數（要先拿鎖）
        if (result == DHT11_OK)
        {
            xSemaphoreTake(xDataMutex, portMAX_DELAY);
            g_temp = dht11_data.temperature_int;
            g_humi = dht11_data.humidity_int;
            xSemaphoreGive(xDataMutex);
            error_count = 0;  // 重置錯誤計數
        }
        else
        {
            error_count++;
            // 3. 若連續錯誤超過 5 次，可加上重新初始化動作
            if (error_count >= 5)
            {
                Set_Pin_Output(DHT11_PORT, DHT11_PIN);
                HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_SET);
                HAL_Delay(500);
                error_count = 0;
            }
        }

        // 4. 等待 2000 ms 再做下一次
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void LCDTask(void *pvParameters)
{
    // 把 buf 放大一点，留足空间
    char buf1[20], buf2[20];
    for (;;)
    {
        int t = g_temp;
        int h = g_humi;

        // 第一行：温度
        HD44780_SetCursor(0, 0);
        snprintf(buf1, sizeof(buf1), "Temp:%3d C       ", t);
        HD44780_PrintStr(buf1);

        // 第二行：先清空，再湿度
        HD44780_SetCursor(0, 1);
        HD44780_PrintStr("                ");  // 16 个空格，清除上一行残影
        HD44780_SetCursor(0, 1);
        snprintf(buf2, sizeof(buf2), "Humi:%3d %%     ", h);
        HD44780_PrintStr(buf2);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


void UARTTask(void *pvParameters)
{
    char buf[64];
    for (;;)
    {
        // 1) 從互斥鎖保護的全域變數取最新值
        xSemaphoreTake(xDataMutex, portMAX_DELAY);
        int t = g_temp;
        int h = g_humi;
        xSemaphoreGive(xDataMutex);

        // 2) 格式化到緩衝區
        int len = sprintf(buf, "Temp:%d C, Humi:%d%%\r\n", t, h);

        // 3) 串口傳輸
        HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, HAL_MAX_DELAY);

        // 4) 1 秒後再做一次
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void AlertTask(void *pvParameters)
{
//	char dbg[64];
    for (;;)
    {
        xSemaphoreTake(xDataMutex, portMAX_DELAY);
        int t = g_temp;
        int h = g_humi;
        xSemaphoreGive(xDataMutex);

        // 改用已宣告的變數
        if (t > temp_threshold || h > humi_threshold){
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
//            int len = sprintf(dbg, "ALERT ON! T:%d H:%d\\r\\n", t, h);
//            HAL_UART_Transmit(&huart2, (uint8_t*)dbg, len, HAL_MAX_DELAY);
        }
        else
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
