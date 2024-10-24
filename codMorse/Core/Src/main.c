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
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
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

I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
void MX_USB_HOST_Process(void);


/* USER CODE BEGIN PFP */
uint32_t Get_Timestamp(void);
void decode_and_print_morse(char *buffer);

#define DEBOUNCE_DELAY 50 // 50 ms debounce delay
#define NO_PRESS_THRESHOLD 5000 // 5000 ms (5 secunde)
#define MAX_BUFFER_SIZE 100 // Maxim 100 caractere in buffer

char buffer[MAX_BUFFER_SIZE]; // Buffer pentru caractere
char buffer_cuvant[MAX_BUFFER_SIZE]; // Buffer pentru caractere
int buffer_index = 0; // Index pentru buffer
int buffer_cuvant_index = 0;
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
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USB_HOST_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);

uint32_t start_time = 0;
uint32_t press_duration = 0;
int button_was_pressed = 0;
uint32_t debounce_time = 0;


printf("\n\r\n\rStart here \n\r");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */

    uint32_t current_time = Get_Timestamp();

    int button_state = HAL_GPIO_ReadPin(GPIOA , GPIO_PIN_0);

    // Debounce logic: only register the button press if the state has been stable for DEBOUNCE_DELAY
        if (button_state == GPIO_PIN_SET && !button_was_pressed && (current_time - debounce_time > DEBOUNCE_DELAY))
        {
            // Button press detected
            debounce_time = current_time;
            start_time = current_time;
            //printf("incepe   %ld \n\r", current_time);
            button_was_pressed = 1;

        }
        else if (button_state == GPIO_PIN_RESET && button_was_pressed && (current_time - debounce_time > DEBOUNCE_DELAY))
        {
            // Button release detected
            debounce_time = current_time;

            // Calculate press duration, accounting for possible timer overflow
            if (current_time >= start_time)
            {
                press_duration = (current_time - start_time);
            }
            else
            {
                // Handle timer overflow
                press_duration = (0xFFFF - start_time + current_time + 1);
            }

            button_was_pressed = 0;
            //printf("termina   %ld \n\r", current_time);
            printf("Buton apasat timp de %ld ms\n\r", press_duration);

            if(press_duration <= 2000)
            {
            	if (buffer_index < MAX_BUFFER_SIZE - 1)
            	        {
            	          buffer[buffer_index++] = '*'; // Adăugăm '*' în buffer
            	        }
            }else if(press_duration <= 4000)
            {
            	if (buffer_index < MAX_BUFFER_SIZE - 1)
            	        {
            	          buffer[buffer_index++] = '-'; // Adăugăm '-' în buffer
            	        }
            }else if (press_duration <= 10000)
            {
            	// Dacă butonul a fost ținut apăsat mai mult de 7 secunde
            	        buffer[buffer_index] = '\0'; // Închidem șirul de caractere
            	        printf("Secventa: %s\n\r", buffer); // Afișăm conținutul bufferului
            	        decode_and_print_morse(buffer);
            	        // Resetăm buffer-ul pentru următoarea secvență
            	        memset(buffer, 0, MAX_BUFFER_SIZE); // Curățăm buffer-ul
            	        buffer_index = 0;
            }else{
            	printf("cuvantul scris este :  %s\n\r" , buffer_cuvant);
            	memset(buffer_cuvant, 0, MAX_BUFFER_SIZE); // Curățăm buffer-ul
            	buffer_cuvant_index = 0;
            }
        }

        HAL_Delay(10);  // Small delay for loop timing

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  __HAL_RCC_TIM1_CLK_ENABLE();
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 16000 - 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
uint32_t Get_Timestamp(void)
{
	 return __HAL_TIM_GET_COUNTER(&htim1);

}

void decode_and_print_morse(char *buffer)
{
    printf("Codul Morse receptionat: %s -> ", buffer);

    // Folosim un switch pentru a decodifica codul Morse
    if (strcmp(buffer, "*-") == 0 && buffer_cuvant_index < MAX_BUFFER_SIZE - 1)
        printf("A\n\r"),buffer_cuvant[buffer_cuvant_index++] = 'A';
    else if (strcmp(buffer, "-***") == 0 && buffer_cuvant_index < MAX_BUFFER_SIZE - 1)
        printf("B\n\r"),buffer_cuvant[buffer_cuvant_index++] = 'B';
    else if (strcmp(buffer, "-*-*") == 0 && buffer_cuvant_index < MAX_BUFFER_SIZE - 1)
        printf("C\n\r"),buffer_cuvant[buffer_cuvant_index++] = 'C';
    else if (strcmp(buffer, "-**") == 0 && buffer_cuvant_index < MAX_BUFFER_SIZE - 1)
        printf("D\n\r"),buffer_cuvant[buffer_cuvant_index++] = 'D';
    else if (strcmp(buffer, "*") == 0 && buffer_cuvant_index < MAX_BUFFER_SIZE - 1)
        printf("E\n\r"),buffer_cuvant[buffer_cuvant_index++] = 'E';
    else if (strcmp(buffer, "**-*") == 0 && buffer_cuvant_index < MAX_BUFFER_SIZE - 1)
        printf("F\n\r"),buffer_cuvant[buffer_cuvant_index++] = 'F';
    else if (strcmp(buffer, "--*") == 0 && buffer_cuvant_index < MAX_BUFFER_SIZE - 1)
        printf("G\n\r"),buffer_cuvant[buffer_cuvant_index++] = 'G';
    else if (strcmp(buffer, "****") == 0 && buffer_cuvant_index < MAX_BUFFER_SIZE - 1)
        printf("H\n\r"),buffer_cuvant[buffer_cuvant_index++] = 'H';
    else if (strcmp(buffer, "**") == 0 && buffer_cuvant_index < MAX_BUFFER_SIZE - 1)
        printf("I\n\r"),buffer_cuvant[buffer_cuvant_index++] = 'I';
    else if (strcmp(buffer, "*---") == 0 && buffer_cuvant_index < MAX_BUFFER_SIZE - 1)
        printf("J\n\r"),buffer_cuvant[buffer_cuvant_index++] = 'J';
    else if (strcmp(buffer, "-*-") == 0 && buffer_cuvant_index < MAX_BUFFER_SIZE - 1)
        printf("K\n\r"),buffer_cuvant[buffer_cuvant_index++] = 'K';
    else if (strcmp(buffer, "*-**") == 0 && buffer_cuvant_index < MAX_BUFFER_SIZE - 1)
        printf("L\n\r"),buffer_cuvant[buffer_cuvant_index++] = 'L';
    else if (strcmp(buffer, "--") == 0 && buffer_cuvant_index < MAX_BUFFER_SIZE - 1)
        printf("M\n\r"),buffer_cuvant[buffer_cuvant_index++] = 'M';
    else if (strcmp(buffer, "-*") == 0 && buffer_cuvant_index < MAX_BUFFER_SIZE - 1)
        printf("N\n\r"),buffer_cuvant[buffer_cuvant_index++] = 'N';
    else if (strcmp(buffer, "---") == 0 && buffer_cuvant_index < MAX_BUFFER_SIZE - 1)
        printf("O\n\r"),buffer_cuvant[buffer_cuvant_index++] = 'O';
    else if (strcmp(buffer, "*--*") == 0 && buffer_cuvant_index < MAX_BUFFER_SIZE - 1)
        printf("P\n\r"),buffer_cuvant[buffer_cuvant_index++] = 'P';
    else if (strcmp(buffer, "--*-") == 0 && buffer_cuvant_index < MAX_BUFFER_SIZE - 1)
        printf("Q\n\r"),buffer_cuvant[buffer_cuvant_index++] = 'Q';
    else if (strcmp(buffer, "*-*") == 0 && buffer_cuvant_index < MAX_BUFFER_SIZE - 1)
        printf("R\n\r"),buffer_cuvant[buffer_cuvant_index++] = 'R';
    else if (strcmp(buffer, "***") == 0 && buffer_cuvant_index < MAX_BUFFER_SIZE - 1)
        printf("S\n\r"),buffer_cuvant[buffer_cuvant_index++] = 'S';
    else if (strcmp(buffer, "-") == 0 && buffer_cuvant_index < MAX_BUFFER_SIZE - 1)
        printf("T\n\r"),buffer_cuvant[buffer_cuvant_index++] = 'T';
    else if (strcmp(buffer, "**-") == 0 && buffer_cuvant_index < MAX_BUFFER_SIZE - 1)
        printf("U\n\r"),buffer_cuvant[buffer_cuvant_index++] = 'U';
    else if (strcmp(buffer, "***-") == 0 && buffer_cuvant_index < MAX_BUFFER_SIZE - 1)
        printf("V\n\r"),buffer_cuvant[buffer_cuvant_index++] = 'V';
    else if (strcmp(buffer, "*--") == 0 && buffer_cuvant_index < MAX_BUFFER_SIZE - 1)
        printf("W\n\r"),buffer_cuvant[buffer_cuvant_index++] = 'W';
    else if (strcmp(buffer, "-**-") == 0 && buffer_cuvant_index < MAX_BUFFER_SIZE - 1)
        printf("X\n\r"),buffer_cuvant[buffer_cuvant_index++] = 'X';
    else if (strcmp(buffer, "-*--") == 0 && buffer_cuvant_index < MAX_BUFFER_SIZE - 1)
        printf("Y\n\r"),buffer_cuvant[buffer_cuvant_index++] = 'Y';
    else if (strcmp(buffer, "--**") == 0 && buffer_cuvant_index < MAX_BUFFER_SIZE - 1)
        printf("Z\n\r"),buffer_cuvant[buffer_cuvant_index++] = 'Z';
    else if (strcmp(buffer, "*----") == 0 && buffer_cuvant_index < MAX_BUFFER_SIZE - 1)
        printf("1\n\r"),buffer_cuvant[buffer_cuvant_index++] = '1';
    else if (strcmp(buffer, "**---") == 0 && buffer_cuvant_index < MAX_BUFFER_SIZE - 1)
        printf("2\n\r"),buffer_cuvant[buffer_cuvant_index++] = '2';
    else if (strcmp(buffer, "***--") == 0 && buffer_cuvant_index < MAX_BUFFER_SIZE - 1)
        printf("3\n\r"),buffer_cuvant[buffer_cuvant_index++] = '3';
    else if (strcmp(buffer, "***-") == 0 && buffer_cuvant_index < MAX_BUFFER_SIZE - 1)
        printf("4\n\r"),buffer_cuvant[buffer_cuvant_index++] = '4';
    else if (strcmp(buffer, "*****") == 0 && buffer_cuvant_index < MAX_BUFFER_SIZE - 1)
        printf("5\n\r"),buffer_cuvant[buffer_cuvant_index++] = '5';
    else if (strcmp(buffer, "-****") == 0 && buffer_cuvant_index < MAX_BUFFER_SIZE - 1)
        printf("6\n\r"),buffer_cuvant[buffer_cuvant_index++] = '6';
    else if (strcmp(buffer, "--***") == 0 && buffer_cuvant_index < MAX_BUFFER_SIZE - 1)
        printf("7\n\r"),buffer_cuvant[buffer_cuvant_index++] = '7';
    else if (strcmp(buffer, "---**") == 0 && buffer_cuvant_index < MAX_BUFFER_SIZE - 1)
        printf("8\n\r"),buffer_cuvant[buffer_cuvant_index++] = '8';
    else if (strcmp(buffer, "----*") == 0 && buffer_cuvant_index < MAX_BUFFER_SIZE - 1)
        printf("9\n\r"),buffer_cuvant[buffer_cuvant_index++] = '9';
    else if (strcmp(buffer, "-----") == 0 && buffer_cuvant_index < MAX_BUFFER_SIZE - 1)
        printf("0\n\r"),buffer_cuvant[buffer_cuvant_index++] = '0';
    else
        printf("Cod invalid\n\r");
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
