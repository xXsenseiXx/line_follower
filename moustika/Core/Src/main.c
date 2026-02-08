/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "fonts.h"
#include "ssd1306.h"
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

uint16_t qtrValues[8];
uint16_t linePos;
uint16_t qtrCalibrated[8];

uint32_t lastPidTime = 0;
uint32_t lastScreenTime = 0;

int LINE_TH = 50;
// External/Global arrays
extern uint16_t qtrValues[8];
uint16_t qtrCalibrated[8];
float Kp = 0.4;
float Kd = 6;
float Ki = 0;

int sharp_right = 0;
int sharp_left = 0;
// Ki is usually not needed for line followers, keeps it simple.

int lastError = 0; // To store previous error for calculating D

int baseSpeed = 800;
int maxSpeed = 1000;
// ==========================================
// MOTOR DRIVER (TB6612FNG)
// ==========================================
// Standby Pin (Must be HIGH to run motors)
#define STBY_PORT       GPIOA
#define STBY_PIN        GPIO_PIN_11

// Motor A (Left) - PWM on TIM3 CH1 (PB4)
#define PWMA_TIM_HANDLE &htim3
#define PWMA_CHANNEL    TIM_CHANNEL_1
#define AIN1_PORT       GPIOA
#define AIN1_PIN        GPIO_PIN_10
#define AIN2_PORT       GPIOA
#define AIN2_PIN        GPIO_PIN_9

// Motor B (Right) - PWM on TIM4 CH1 (PB6)
#define PWMB_TIM_HANDLE &htim4
#define PWMB_CHANNEL    TIM_CHANNEL_1
#define BIN1_PORT       GPIOA
#define BIN1_PIN        GPIO_PIN_12
#define BIN2_PORT       GPIOA
#define BIN2_PIN        GPIO_PIN_15  // WARNING: JTAG Pin (Disable JTAG in CubeMX!)

// ==========================================
// USER INTERFACE (Buttons)
// ==========================================
// Wired to GPIOB, assuming Pull-UP input (Connect button to GND)
#define BTN_PORT        GPIOB
#define BTN_UP_PIN      GPIO_PIN_12
#define BTN_DOWN_PIN    GPIO_PIN_13
#define BTN_RIGHT_PIN   GPIO_PIN_14
#define BTN_LEFT_PIN    GPIO_PIN_15

// ==========================================
// SENSORS (QTR-8A)
// ==========================================
// Connected to ADC1 IN0 -> IN7 (PA0 -> PA7)
// No defines needed here if using DMA array,
// but good to know for reference:
#define SENSOR_PORT     GPIOA
#define SENSOR_1_PIN    GPIO_PIN_0
#define SENSOR_8_PIN    GPIO_PIN_7

// ==========================================
// EXTRAS
// ==========================================
#define BUZZER_PORT     GPIOB
#define BUZZER_PIN      GPIO_PIN_9  // TIM11 CH1
#define OLED_I2C_PORT   GPIOB
#define OLED_SCL_PIN    GPIO_PIN_8
#define OLED_SDA_PIN    GPIO_PIN_7

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim11;

/* USER CODE BEGIN PV */
typedef enum {
    STATE_MENU,
	STATE_START,
	STATE_PID

} SystemState;

SystemState currentState = STATE_MENU;
int menuIndex = 0;

const char* menuItems[] = {
    "1. START",
	"2. PID "

};

uint8_t updateScreen = 1;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM11_Init(void);
/* USER CODE BEGIN PFP */

void SetMotorA(int speed);
void SetMotorB(int speed);
void EnableMotors();
void DisableMotors();
uint8_t IsPressed(GPIO_TypeDef *Port, uint16_t Pin);
uint16_t CalculateLinePosition(void);

void DrawGraphScreen(uint16_t position);

void Run_PID(void);
void calibrateQTR(void);
void SetupTrackCalibration(void);

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
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */
  //HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  //setMotorSpeed(500);
  SSD1306_Init();
  SSD1306_Fill(0); // Clear black
  SSD1306_UpdateScreen();

  // 2. Start PWM Timers
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  EnableMotors();

  if(HAL_ADC_Start_DMA(&hadc1, (uint32_t*)qtrValues, 8) != HAL_OK)
  {
      // DMA Start Error Handler (Optional: Print to OLED)
      SSD1306_GotoXY(0,0);
      SSD1306_Puts("DMA ERR", &Font_7x10, 1);
      SSD1306_UpdateScreen();
      Error_Handler();
  }


  SetupTrackCalibration();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

      if (HAL_GetTick() - lastPidTime >= 1)
      {
          lastPidTime = HAL_GetTick();

          calibrateQTR();
          linePos = CalculateLinePosition();
          Run_PID();

          int leftSide   = (qtrCalibrated[0] > LINE_TH) + (qtrCalibrated[1] > LINE_TH) + (qtrCalibrated[2] > LINE_TH);
          int rightSide  = (qtrCalibrated[5] > LINE_TH) + (qtrCalibrated[6] > LINE_TH) + (qtrCalibrated[7] > LINE_TH);
          int center     = (qtrCalibrated[3] > LINE_TH) + (qtrCalibrated[4] > LINE_TH);

          int totalActive = leftSide + rightSide + center;

          if(leftSide >= 2 && rightSide == 0) {
              sharp_left = 1;
              sharp_right = 0; // Clear the other flag to be safe
          }
          if(rightSide >= 2 && leftSide == 0) {
              sharp_left = 0;
              sharp_right = 1; // Clear the other flag to be safe
          }

          if (totalActive <= 1)
                    {
                        // Brake briefly to stop forward momentum before spinning
                        SetMotorA(0);
                        SetMotorB(0);
                        HAL_Delay(50);

                        while (1)
                        {
                            calibrateQTR();
                            linePos = CalculateLinePosition(); // Just to get raw values updated

                            // EXIT CONDITION: Center sensors see the line
                            if (qtrCalibrated[3] > LINE_TH || qtrCalibrated[4] > LINE_TH) {
                                // Reset flags and exit recovery
                                sharp_left = 0;
                                sharp_right = 0;
                                lastError = 0; // Reset PID memory
                                break;
                            }

                            // MOVEMENT LOGIC (Must use ELSE IF)
                            if(sharp_left) {
                                // We remembered a left turn -> Spin Left
                                SetMotorA(-700); // Reduced from 1000 to catch line easier
                                SetMotorB(700);
                            }
                            else if(sharp_right) {
                                // We remembered a right turn -> Spin Right
                                SetMotorA(700);
                                SetMotorB(-700);
                            }
                            else {
                                // No flag set? Use PID History (Standard Gap)
                                if(lastError > 0) {
                                     // Lost to the Right -> Spin Right
                                     SetMotorA(700);
                                     SetMotorB(-700);
                                }
                                else {
                                     // Lost to the Left -> Spin Left
                                     SetMotorA(-700);
                                     SetMotorB(700);
                                }
                            }
                            // Small delay to prevent CPU hogging in while loop (optional)
                            // HAL_Delay(1);
                        }
                    }

      }

      if (HAL_GetTick() - lastScreenTime >= 100)
      {
          lastScreenTime = HAL_GetTick();
          DrawGraphScreen(linePos);
      }

      // 3. Delay
      //HAL_Delay(50);

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 8;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.ClockSpeed = 400000;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 4;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 4;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 0;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 65535;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */
  HAL_TIM_MspPostInit(&htim11);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, AIN2_Pin|AIN1_Pin|STBY_Pin|BIN1_Pin
                          |BIN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : up_Pin down_Pin right_Pin left_Pin */
  GPIO_InitStruct.Pin = up_Pin|down_Pin|right_Pin|left_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : AIN2_Pin AIN1_Pin STBY_Pin BIN1_Pin
                           BIN2_Pin */
  GPIO_InitStruct.Pin = AIN2_Pin|AIN1_Pin|STBY_Pin|BIN1_Pin
                          |BIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void SetMotorA(int speed) {
    // 1. Set Direction
    if (speed > 0) {
        HAL_GPIO_WritePin(AIN1_PORT, AIN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(AIN2_PORT, AIN2_PIN, GPIO_PIN_RESET);
    } else if (speed < 0) {
        HAL_GPIO_WritePin(AIN1_PORT, AIN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_PORT, AIN2_PIN, GPIO_PIN_SET);
        speed = -speed; // Make positive for PWM
    } else {
        HAL_GPIO_WritePin(AIN1_PORT, AIN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(AIN2_PORT, AIN2_PIN, GPIO_PIN_SET); // Brake
    }

    // 2. Limit Speed
    if (speed > 1000) speed = 1000;

    // 3. Set PWM
    __HAL_TIM_SET_COMPARE(PWMA_TIM_HANDLE, PWMA_CHANNEL, speed);
}

void SetMotorB(int speed) {
    // 1. Set Direction
    if (speed > 0) {
        HAL_GPIO_WritePin(BIN1_PORT, BIN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BIN2_PORT, BIN2_PIN, GPIO_PIN_RESET);
    } else if (speed < 0) {
        HAL_GPIO_WritePin(BIN1_PORT, BIN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BIN2_PORT, BIN2_PIN, GPIO_PIN_SET);
        speed = -speed;
    } else {
        HAL_GPIO_WritePin(BIN1_PORT, BIN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BIN2_PORT, BIN2_PIN, GPIO_PIN_SET); // Brake
    }

    // 2. Limit Speed
    if (speed > 1000) speed = 1000;

    // 3. Set PWM
    __HAL_TIM_SET_COMPARE(PWMB_TIM_HANDLE, PWMB_CHANNEL, speed);
}

void EnableMotors() {
    HAL_GPIO_WritePin(STBY_PORT, STBY_PIN, GPIO_PIN_SET);
}

void DisableMotors() {
    HAL_GPIO_WritePin(STBY_PORT, STBY_PIN, GPIO_PIN_RESET);
}

uint8_t IsPressed(GPIO_TypeDef *Port, uint16_t Pin) {
  if (HAL_GPIO_ReadPin(Port, Pin) == 0) {
    HAL_Delay(50);
    if (HAL_GPIO_ReadPin(Port, Pin) == 0) {
      while (HAL_GPIO_ReadPin(Port, Pin) == 0)
        ;
      return 1;
    }
  }
  return 0;
}


uint16_t value_;



uint16_t qtrCalibrated[8];


/*int  ADC_MAX =4095;
int  NOISE_FLOOR = 660 ;   // noise threshold
int FULL_SPREAD = 200 ;   // sensors must be close to each other
int  FULL_WHITE_MAX = 650 ;   // average below → white
int FULL_BLACK_MIN = 3800  ; // average above → black*/

int ADC_MAX = 4095;
int NOISE_FLOOR = 0;      // Will be calculated
int FULL_SPREAD = 0;      // Will be calculated
int FULL_WHITE_MAX = 0;   // Will be calculated
int FULL_BLACK_MIN = 0;   // Will be calculated
int BLACK_THRESHOLD = 0;
int BLACK_THRESHOLD_pos = 0;


void calibrateQTR(void)
{
    uint16_t min = qtrValues[0];
    uint16_t max = qtrValues[0];
    uint32_t sum = 0;

    // Pass 1: Statistics
    for (int i = 0; i < 8; i++)
    {
        uint16_t v = qtrValues[i];
        if (v < min) min = v;
        if (v > max) max = v;
        sum += v;
    }

    uint16_t avg = sum / 8;
    uint16_t spread = max - min;

    // ===== PRIORITY OVERRIDE =====

    // Full white (Low Spread AND Low Average) -> force LOW
    if (spread < FULL_SPREAD && avg < FULL_WHITE_MAX)
    {
        memset(qtrCalibrated, 0, sizeof(qtrCalibrated));
        return;
    }

    // Full black (Low Spread AND High Average) -> force HIGH
    if (spread < FULL_SPREAD && avg > FULL_BLACK_MIN)
    {
        for (int i = 0; i < 8; i++)
            qtrCalibrated[i] = ADC_MAX;
        return;
    }

    // ===== NORMAL CALIBRATION =====

    if (max - min < NOISE_FLOOR)
    {
        memset(qtrCalibrated, 0, sizeof(qtrCalibrated));
        return;
    }

    for (int i = 0; i < 8; i++)
    {
        // ===== THE HARD FILTER =====
        // If this specific sensor is not "Dark Enough" to be part of the line,
        // we kill it immediately.
        // This removes the "White Noise" flickering completely.
        if (qtrValues[i] < BLACK_THRESHOLD || qtrValues[i] > BLACK_THRESHOLD_pos)
        {
             qtrCalibrated[i] = 0;
             continue;
        }

        // Standard Normalization
        int32_t v = qtrValues[i] - min;

        // Extra noise check
        if (v < NOISE_FLOOR)
        {
            qtrCalibrated[i] = 0;
            continue;
        }

        uint32_t scaled = (v * ADC_MAX) / (max - min);

        // Square for contrast
        scaled = (scaled * scaled) / ADC_MAX;

        // Clamp
        if (scaled > ADC_MAX) scaled = ADC_MAX;

        qtrCalibrated[i] = (uint16_t)scaled;
    }
}

uint16_t CalculateLinePosition(void)
{
    uint32_t weightedSum = 0;
    uint32_t totalSum = 0;
    static uint16_t lastPosition = 3500;

    // 1. Line Isolation Logic
    // We only accept sensors that are "connected" to the last known position.
    // Sensors are at 0, 1000, 2000... 7000.
    // If we were at 3500, valid sensors are 2, 3, 4, 5.
    // Sensor 7 (Right Wing) would be distance 3500 -> IGNORED.

    int activeSensors = 0;

    for(int i = 0; i < 8; i++)
    {
        uint16_t val = qtrCalibrated[i];

        // Skip weak signals
        if (val < 100) continue;

        // TRACKING LOGIC:
        // Calculate physical position of this specific sensor
        int sensorPos = i * 1000;

        // Calculate distance from previous robot position
        int distance = sensorPos - lastPosition;
        if(distance < 0) distance = -distance; // Absolute value

        // Threshold: 2500 means "2.5 sensors away".
        // If a sensor is more than 2 spots away from our line, it's a cross-line/noise.
        // UNLESS: We haven't seen a line for a while (totalSum == 0), then we accept anything.
        if (distance > 2500 && totalSum > 100)
        {
            // Ignore this sensor (It's the "Right Wing" 000110[11])
            val = 0;
        }

        totalSum += val;
        weightedSum += (val * sensorPos);
        if(val > 0) activeSensors++;
    }

    // 2. Lost Line Logic
    if (totalSum < 500) {
        // If we see NOTHING, we don't update lastPosition.
        // We return the last known position so PID keeps turning in that direction.
        return lastPosition;
    }

    uint16_t currentPosition = weightedSum / totalSum;
    lastPosition = currentPosition;

    return currentPosition;
}
void DrawGraphScreen(uint16_t position)
{
    char lcdBuffer[20];

    // 1. Clear Screen buffer
    SSD1306_Fill(0);

    // 2. Print Stats
    // Print Position
    sprintf(lcdBuffer, "Pos: %d", position);
    SSD1306_GotoXY(0, 0);
    SSD1306_Puts(lcdBuffer, &Font_7x10, 1);

    //sprintf(lcdBuffer, "Err: %d", error);
    sprintf(lcdBuffer, "V: %d", qtrValues[0]);
    SSD1306_GotoXY(65, 0);
    SSD1306_Puts(lcdBuffer, &Font_7x10, 1);

    // 3. Draw Bar Graphs
    for (int i = 0; i < 8; i++)
    {
        // Scaling: 4095 (Max ADC) -> 48 (Max Height)
        uint8_t barHeight = (qtrCalibrated[i] * 48) / 4095;
        //uint8_t barHeight = (qtrValues[i] * 48) / 4095;

        if(barHeight > 48) barHeight = 48;

        uint8_t xPos = i * 16;
        uint8_t yPos = 64 - barHeight;

        // Draw solid bar
        SSD1306_DrawFilledRectangle(xPos, yPos, 14, barHeight, 1);
    }

    uint8_t cursorX = (position * 128) / 7000;

    SSD1306_DrawLine(cursorX, 15, cursorX, 64, 0);

    // Draw a marker at the bottom
    SSD1306_DrawPixel(cursorX, 0, 1);
    SSD1306_DrawPixel(cursorX-1, 0, 1);
    SSD1306_DrawPixel(cursorX+1, 0, 1);

    SSD1306_UpdateScreen();
}
int I;
void Run_PID(void)
{
    uint16_t position = CalculateLinePosition();
    int error = position - 3500;

    int P = error;
    int D = error - lastError;

    // INTEGRAL with WINDUP GUARD
    I = I + error;
    // Cap I at approx 1/3 of max speed worth of correction
    if (I > 10000) I = 10000;
    if (I < -10000) I = -10000;

    lastError = error;

    // Calculate Correction
    // Note: Ki usually needs to be TINY (e.g., 0.001) for lines
    float motorSpeed = (Kp * P) + (Kd * D) + (Ki * I);

    int rightMotorSpeed = baseSpeed - motorSpeed;
    int leftMotorSpeed = baseSpeed + motorSpeed;

    // Constrain
    if (rightMotorSpeed > maxSpeed) rightMotorSpeed = maxSpeed;
    if (leftMotorSpeed > maxSpeed) leftMotorSpeed = maxSpeed;
    if (rightMotorSpeed < -maxSpeed) rightMotorSpeed = -maxSpeed;
    if (leftMotorSpeed < -maxSpeed) leftMotorSpeed = -maxSpeed;

    SetMotorA(leftMotorSpeed);
    SetMotorB(rightMotorSpeed);
}

void SetupTrackCalibration(void)
{
    uint16_t globalMin = 4095;
    uint16_t globalMax = 0;

    memset(qtrCalibrated, 0, sizeof(qtrCalibrated));
    int calibration_loops = 1000000;

    while(calibration_loops > 0)
    {
        uint32_t currentSum = 0;
        uint16_t currentMin = 4095;
        uint16_t currentMax = 0;

        for(int i = 0; i < 8; i++)
        {
            uint16_t val = qtrValues[i];
            currentSum += val;
            if(val < currentMin) currentMin = val;
            if(val > currentMax) currentMax = val;
        }

        uint16_t currentAvg = currentSum / 8;
        if(currentAvg < globalMin) globalMin = currentAvg;
        if(currentAvg > globalMax) globalMax = currentAvg;

        calibration_loops--;
    }

    uint16_t dynamicRange = globalMax - globalMin;

    if (dynamicRange < 500) {
        globalMin = 100;
        globalMax = 3800;
        dynamicRange = 3700;
    }

    FULL_WHITE_MAX = globalMin + (dynamicRange * 20 / 100);
    FULL_BLACK_MIN = globalMax - (dynamicRange * 20 / 100);
    NOISE_FLOOR = dynamicRange * 15 / 100;
    FULL_SPREAD = dynamicRange * 20 / 100;

    // ===== THE FILTER FIX =====
    // We set the threshold relative to the BLACKEST point found.
    // We allow a margin (e.g. 800) so we can still see the edges of the line.
    // If you want it stricter, decrease 800 to 400.
    // Do NOT go below 200 or PID will fail.
    BLACK_THRESHOLD = globalMax - 600;
    BLACK_THRESHOLD_pos = globalMax + 100;
}/* USER CODE END 4 */

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
