/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h> // atoi fonksiyonu için
//deneme

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

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// --- Motor Nesnesi Yapısı ---
typedef struct {
    uint8_t voltage_mode;       // 5V veya 12V
    uint16_t max_pwm_limit;     // PWM üst limiti

    // Donanım Bağlantıları
    GPIO_TypeDef* port_in1;
    uint16_t pin_in1;
    GPIO_TypeDef* port_in2;
    uint16_t pin_in2;
    TIM_HandleTypeDef* htim;
    uint32_t channel;

    // Hedef ve Durum
    volatile uint16_t target_pwm;
    volatile int8_t target_dir;     // 1: İleri, -1: Geri, 0: Fren

    // Rampa Değişkenleri
    float current_pwm;
    int8_t current_dir;
} Motor_t;

Motor_t motors[4]; // 4 Motorumuz

// --- UART ve Tampon ---
#define RX_BUFFER_SIZE 32
uint8_t rx_temp;
uint8_t rx_buffer[RX_BUFFER_SIZE];
uint8_t rx_index = 0;
volatile uint8_t data_ready = 0; // Veri geldi bayrağı

char tx_buffer[128]; // Gönderim tamponu

// --- Sistem Durumu ---
uint8_t system_state = 0; // 0: Kurulum Modu, 1: Çalışma Modu
uint8_t config_step = 0;  // Hangi motoru soruyoruz?

// --- Ayarlar ---
#define PWM_PERIOD 6799
#define RAMP_STEP 50.0f  // Hızlanma adımı
#define RAMP_DELAY 10    // Döngü gecikmesi (ms)

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void Motor_Init_Structs(void);
void Motor_SetVoltage(uint8_t index, uint8_t voltage);
void Motor_Update(void);
void Ask_Next_Config(void);
void Process_Command(char* cmd);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// 1. Structları Donanımla Eşleştirme
void Motor_Init_Structs(void)
{
    // MOTOR 1
    motors[0].htim = &htim2; motors[0].channel = TIM_CHANNEL_1;
    motors[0].port_in1 = M1_IN1_GPIO_Port; motors[0].pin_in1 = M1_IN1_Pin;
    motors[0].port_in2 = M1_IN2_GPIO_Port; motors[0].pin_in2 = M1_IN2_Pin;

    // MOTOR 2
    motors[1].htim = &htim2; motors[1].channel = TIM_CHANNEL_2;
    motors[1].port_in1 = M2_IN1_GPIO_Port; motors[1].pin_in1 = M2_IN1_Pin;
    motors[1].port_in2 = M2_IN2_GPIO_Port; motors[1].pin_in2 = M2_IN2_Pin;

    // MOTOR 3
    motors[2].htim = &htim2; motors[2].channel = TIM_CHANNEL_3;
    motors[2].port_in1 = M3_IN1_GPIO_Port; motors[2].pin_in1 = M3_IN1_Pin;
    motors[2].port_in2 = M3_IN2_GPIO_Port; motors[2].pin_in2 = M3_IN2_Pin;

    // MOTOR 4
    motors[3].htim = &htim2; motors[3].channel = TIM_CHANNEL_4;
    motors[3].port_in1 = M4_IN1_GPIO_Port; motors[3].pin_in1 = M4_IN1_Pin;
    motors[3].port_in2 = M4_IN2_GPIO_Port; motors[3].pin_in2 = M4_IN2_Pin;
}

// 2. Voltaj Limiti Ayarlama
void Motor_SetVoltage(uint8_t index, uint8_t voltage)
{
    if(index >= 4) return;
    motors[index].voltage_mode = voltage;

    if(voltage == 5) {
        // 12V girişten 5V çıkış (%41.6)
        motors[index].max_pwm_limit = (uint16_t)(PWM_PERIOD * 0.416f);
    } else {
        motors[index].max_pwm_limit = PWM_PERIOD;
    }
}

// 3. Kullanıcıya Soru Sorma (Kurulum Sihirbazı)
void Ask_Next_Config(void)
{
    if (config_step < 4) {
        sprintf(tx_buffer, ">> Motor %d Calisma Voltaji? (5 veya 12): ", config_step + 1);
        HAL_UART_Transmit(&huart2, (uint8_t*)tx_buffer, strlen(tx_buffer), 100);
    } else {
        system_state = 1; // Çalışma moduna geç
        sprintf(tx_buffer, "\r\n--- KURULUM TAMAMLANDI ---\r\nHazir. Ornek: 'M1 F 50' veya 'A F'\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)tx_buffer, strlen(tx_buffer), 100);
    }
}

// 4. Gelen Veriyi İşleme
void Process_Command(char* cmd)
{
    // Komutun başındaki boşlukları ve görünmez karakterleri atla
    char* p = cmd;
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;

    // Eğer boş komutsa çık
    if (*p == 0) return;

    // --- MOD 0: KURULUM ---
    if (system_state == 0)
    {
        int voltage = atoi(p);
        if (voltage == 5 || voltage == 12) {
            Motor_SetVoltage(config_step, voltage);
            sprintf(tx_buffer, "   -> M%d: %dV OK.\r\n", config_step + 1, voltage);
            HAL_UART_Transmit(&huart2, (uint8_t*)tx_buffer, strlen(tx_buffer), 100);
            config_step++;
            Ask_Next_Config();
        } else {
            sprintf(tx_buffer, "   HATALI! Sadece 5 veya 12 giriniz.\r\n");
            HAL_UART_Transmit(&huart2, (uint8_t*)tx_buffer, strlen(tx_buffer), 100);
            Ask_Next_Config();
        }
    }
    // --- MOD 1: ÇALIŞMA ---
    else
    {
        // Toplu Komutlar
        if (p[0] == 'A') {
            char action;
            if (sscanf(p, "A %c", &action) == 1) {
                for(int i=0; i<4; i++) {
                    if (action == 'B') { motors[i].target_dir = 0; motors[i].target_pwm = 0; }
                    else if (action == 'F') motors[i].target_dir = 1;
                    else if (action == 'R') motors[i].target_dir = -1;
                }
                HAL_UART_Transmit(&huart2, (uint8_t*)"OK\r\n", 4, 100);
            }
        }
        // Tekil Komutlar
        else if (p[0] == 'M') {
            int m_id, spd_perc;
            char dir_chr;
            if (sscanf(p, "M%d %c %d", &m_id, &dir_chr, &spd_perc) == 3) {
                if (m_id >= 1 && m_id <= 4) {
                    int idx = m_id - 1;
                    if (dir_chr == 'F') motors[idx].target_dir = 1;
                    else if (dir_chr == 'R') motors[idx].target_dir = -1;
                    else if (dir_chr == 'B') motors[idx].target_dir = 0;

                    uint32_t calc = (motors[idx].max_pwm_limit * spd_perc) / 100;
                    motors[idx].target_pwm = (uint16_t)calc;

                    HAL_UART_Transmit(&huart2, (uint8_t*)"OK\r\n", 4, 100);
                }
            }
        }
    }


    // İşimiz bitti, buffer'ı sıfırla ki sonraki komuta "hayalet" karakter kalmasın.
    memset(rx_buffer, 0, RX_BUFFER_SIZE);
}

// 5. Motor Güncelleme (Rampa ve Güvenlik)
void Motor_Update(void)
{
    for(int i = 0; i < 4; i++)
    {
        // Yön Değişimi Güvenliği
        if (motors[i].current_dir != motors[i].target_dir)
        {
            if (motors[i].current_pwm > 0) {
                motors[i].current_pwm -= RAMP_STEP * 2; // Hızlı yavaşla
                if (motors[i].current_pwm < 0) motors[i].current_pwm = 0;
            } else {
                motors[i].current_dir = motors[i].target_dir; // Durunca yön değiştir
            }
        }
        else // Yön aynı ise normal rampa
        {
            if (motors[i].current_pwm < motors[i].target_pwm) {
                motors[i].current_pwm += RAMP_STEP;
                if(motors[i].current_pwm > motors[i].target_pwm) motors[i].current_pwm = motors[i].target_pwm;
            }
            else if (motors[i].current_pwm > motors[i].target_pwm) {
                motors[i].current_pwm -= RAMP_STEP;
                if(motors[i].current_pwm < 0) motors[i].current_pwm = 0;
            }
        }

        // Donanıma Uygula (Pinler)
        if (motors[i].current_dir == 1) { // İleri
            HAL_GPIO_WritePin(motors[i].port_in1, motors[i].pin_in1, GPIO_PIN_SET);
            HAL_GPIO_WritePin(motors[i].port_in2, motors[i].pin_in2, GPIO_PIN_RESET);
        } else if (motors[i].current_dir == -1) { // Geri
            HAL_GPIO_WritePin(motors[i].port_in1, motors[i].pin_in1, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(motors[i].port_in2, motors[i].pin_in2, GPIO_PIN_SET);
        } else { // Fren
            HAL_GPIO_WritePin(motors[i].port_in1, motors[i].pin_in1, GPIO_PIN_SET);
            HAL_GPIO_WritePin(motors[i].port_in2, motors[i].pin_in2, GPIO_PIN_SET);
        }

        // Donanıma Uygula (PWM)
        __HAL_TIM_SET_COMPARE(motors[i].htim, motors[i].channel, (uint16_t)motors[i].current_pwm);
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
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  Motor_Init_Structs();

    // PWM Başlat
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

    // UART Dinle
    HAL_UART_Receive_IT(&huart2, &rx_temp, 1);

    // Sistemi Başlat (Ekran Temizle ve İlk Soruyu Sor)
    HAL_UART_Transmit(&huart2, (uint8_t*)"\033[2J\033[H", 7, 100); // Ekran temizle
    HAL_UART_Transmit(&huart2, (uint8_t*)"=== SISTEM BASLADI ===\r\n", 24, 100);
    Ask_Next_Config();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if (data_ready) {
	            Process_Command((char*)rx_buffer);
	            data_ready = 0;
	        }

	        Motor_Update();
	        HAL_Delay(RAMP_DELAY);
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  hi2c1.Init.Timing = 0x40B285C2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 6799;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, M1_IN2_Pin|M1_IN1_Pin|M2_IN2_Pin|M2_IN1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, M3_IN2_Pin|M3_IN1_Pin|M4_IN2_Pin|M4_IN1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : M1_IN2_Pin M1_IN1_Pin M2_IN2_Pin M2_IN1_Pin
                           LED_STATUS_Pin */
  GPIO_InitStruct.Pin = M1_IN2_Pin|M1_IN1_Pin|M2_IN2_Pin|M2_IN1_Pin
                          |LED_STATUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : M3_IN2_Pin M3_IN1_Pin M4_IN2_Pin M4_IN1_Pin */
  GPIO_InitStruct.Pin = M3_IN2_Pin|M3_IN1_Pin|M4_IN2_Pin|M4_IN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        // Gelen karakter 'Enter' (\n veya \r) mı?
        if (rx_temp == '\n' || rx_temp == '\r')
        {
            // Eğer buffer doluysa (gerçekten bir komut yazılmışsa)
            if (rx_index > 0) {
                rx_buffer[rx_index] = '\0'; // Stringi kapat
                data_ready = 1;             // Main'e haber ver
                rx_index = 0;               // Sayacı sıfırla
            }
            // Eğer buffer boşsa (rx_index == 0) ve Enter geldiyse;
            // Bu, CR+LF ikilisinin ikinci parçasıdır veya boş Enter'dır.
            // HİÇBİR ŞEY YAPMA (Görmezden gel).
        }
        else
        {
            // Normal karakter geldiyse ve yer varsa kaydet
            if (rx_index < RX_BUFFER_SIZE - 1) {
                rx_buffer[rx_index++] = rx_temp;
            }
        }

        // Dinlemeye devam et
        HAL_UART_Receive_IT(&huart2, &rx_temp, 1);
    }
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
#ifdef USE_FULL_ASSERT
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
