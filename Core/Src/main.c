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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"

#include "nhd0420dzw.h"
#include "serial.h"
#include "control.h"
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
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
/* System clock ticks per millisecond */
uint32_t msec_ticks;
/* System clock ticks per microsecond */
uint32_t usec_ticks;

/* Sensor output data */
struct bmp5_sensor_data bmp5_data[2];
ms5525dso_data_t ms5525_data;


/* Device handles */
bmp580_ref bmp5_dev1 = NULL;
bmp580_ref bmp5_dev2 = NULL;
ms5525_ref ms5525_dev = NULL;
nhd0420_ref lcd_dev = NULL;

/*
 * Integral status of all sensors.
 */
HAL_StatusTypeDef sensors_active = HAL_OK;

device_status_t device_status_table[DEV_STATUS_COUNT] = {
    [DEV_SENSOR_BMP_1] = {
	HAL_ERROR,
	&hi2c1,
	BMP5_LED1_GPIO_Port,
	BMP5_LED1_Pin,
    },
    [DEV_SENSOR_BMP_2] = {
	HAL_ERROR,
	&hi2c1,
	BMP5_LED2_GPIO_Port,
	BMP5_LED2_Pin,
    },
    [DEV_SENSOR_MS5525] = {
	HAL_ERROR,
	&hi2c2,
	MS5525_LED_GPIO_Port,
	MS5525_LED_Pin,
    },
};


/* Button states */
#define UNITS_SWITCH_IS_MOMENTARY 0

enum {
  BTN_UNITS = 0,
  BTN_COUNT,
};

static uint8_t buttons_states[BTN_COUNT] = { 0 };
static uint8_t button_pressed[BTN_COUNT] = { 0 };

static GPIO_TypeDef *buttons_ports[BTN_COUNT] = { SW_UNITS_GPIO_Port };
static uint32_t buttons_pins[BTN_COUNT] = { SW_UNITS_Pin };

/* Display units */
enum {
  UNITS_SET1, /* Pa */
  UNITS_SET2, /* PSI, inHg */
  UNITS_SET_COUNT,
};

static uint8_t active_units = UNITS_SET1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
void read_sensors(void);
void update_screen(void);
void update_indication_leds(void);
void check_reset_i2c_bus(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * TIM1 is setup to run at 1kHz, counting milliseconds.
 * This is 16-bit timer used to time short intervals needed by sensors.
 */
uint16_t ms_clock(void)
{
  return TIM1->CNT;
}

/**
 * TIM2 is setup to run at 1MHz, counting microseconds.
 * This is 16-bit timer used to time short intervals needed by sensors.
 */
uint16_t us_clock(void)
{
  return TIM2->CNT;
}

void us_delay(uint32_t delay)
{
  uint16_t cnt = LL_TIM_GetCounter(TIM2);
  uint16_t new_cnt = cnt + delay;
  LL_TIM_OC_SetCompareCH1(TIM2, new_cnt);
  while (LL_TIM_GetCounter(TIM2) - cnt < delay) {
//      __WFI();
  }
}

/* Button processing function.
 * Button press action is registered upon release.
 * This function does not do jitter filtering.
 * Registered press action is stored in the button_pressed[] array.
 * The reader is responsible for clearing the flag after reading.
 *  */
static void check_button_presses(void)
{
  unsigned i;
  for (i = 0; i < BTN_COUNT; i++) {
      uint32_t pinstate = LL_GPIO_IsInputPinSet(buttons_ports[i], buttons_pins[i]);

      if (pinstate ^ buttons_states[i]) {
	  button_pressed[i] = buttons_states[i];
      }

      buttons_states[i] = pinstate;
  }
}

/* Units selection */
static void check_units_button(void)
{
#if UNITS_SWITCH_IS_MOMENTARY
  if (button_pressed[BTN_UNITS]) {
      button_pressed[BTN_UNITS] = 0;
      active_units += 1;
      if (active_units > UNITS_SET_COUNT) {
	  active_units = UNITS_SET1;
      }
  }
#else
  active_units = buttons_states[BTN_UNITS] ? UNITS_SET1 : UNITS_SET2;
#endif
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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
//  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH3);
  LL_TIM_EnableCounter(TIM1);
  LL_TIM_EnableCounter(TIM2);
  LL_TIM_EnableCounter(TIM3);
  LL_TIM_EnableCounter(TIM4);

  load_control_parameters(&hi2c2);
//  HAL_GPIO_WritePin(LED_PIN_GPIO_Port, LED_PIN_Pin, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  lcd_dev = nhd0420_setup(&hspi1);

  uint16_t usec_counter = us_clock();
  uint16_t l_delta = 0;
  uint32_t update_screen_timer = 0;

  while (1)
  {
      l_delta = us_clock() - usec_counter;
      usec_counter += l_delta;

      update_screen_timer += l_delta;
      if (update_screen_timer > 500000) {

	  check_save_control_parameters(&hi2c2);
	  check_units_button();

	  /* Update screen every 0.5 second */
	  update_screen_timer -= 500000;
	  update_screen();

	  /* Indicate overall system health using the on-board led.
	   * Solid green = Good, blinking green = Error. */
	  if (HAL_OK == sensors_active) {
	      LL_GPIO_ResetOutputPin(LED_PIN_GPIO_Port, LED_PIN_Pin);
	  } else {
	      LL_GPIO_TogglePin(LED_PIN_GPIO_Port, LED_PIN_Pin);
	  }
      }

      read_sensors();
      run_control_loop();
      update_indication_leds();
      check_reset_i2c_bus();
      check_button_presses();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      process_serial_input();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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
  /* When assigning Prescaler divide by 2 to fit in 16 bits. ClockDivision must be also set to DIV2 */
  msec_ticks = (HAL_RCC_GetHCLKFreq() / 1000);
  /* USER CODE END TIM1_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  TIM_InitStruct.Prescaler = msec_ticks/2;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 65535;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV2;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM1, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM1);
  LL_TIM_SetClockSource(TIM1, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_UPDATE);
  LL_TIM_DisableMasterSlaveMode(TIM1);
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */
  usec_ticks = (HAL_RCC_GetHCLKFreq() / 1000000);
  /* USER CODE END TIM2_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  TIM_InitStruct.Prescaler = usec_ticks-1;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 65535;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM2, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM2);
  LL_TIM_SetClockSource(TIM2, LL_TIM_CLOCKSOURCE_INTERNAL);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_TOGGLE;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 0;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  LL_TIM_OC_Init(TIM2, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM2, LL_TIM_CHANNEL_CH1);
  LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_OC1REF);
  LL_TIM_DisableMasterSlaveMode(TIM2);
  LL_TIM_OC_EnablePreload(TIM2, LL_TIM_CHANNEL_CH1);
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  TIM_InitStruct.Prescaler = 1;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 500;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM3, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM3);
  LL_TIM_SetTriggerInput(TIM3, LL_TIM_TS_ITR0);
  LL_TIM_SetClockSource(TIM3, LL_TIM_CLOCKSOURCE_EXT_MODE1);
  LL_TIM_DisableIT_TRIG(TIM3);
  LL_TIM_DisableDMAReq_TRIG(TIM3);
  LL_TIM_SetTriggerOutput(TIM3, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM3);
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 65535;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM4, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM4);
  LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH1);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 0;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM4, LL_TIM_CHANNEL_CH1);
  LL_TIM_SetTriggerInput(TIM4, LL_TIM_TS_ITR1);
  LL_TIM_SetClockSource(TIM4, LL_TIM_CLOCKSOURCE_EXT_MODE1);
  LL_TIM_DisableIT_TRIG(TIM4);
  LL_TIM_DisableDMAReq_TRIG(TIM4);
  LL_TIM_SetTriggerOutput(TIM4, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM4);
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
  /**TIM4 GPIO Configuration
  PB6   ------> TIM4_CH1
  */
  GPIO_InitStruct.Pin = STEPPER_STEP_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(STEPPER_STEP_GPIO_Port, &GPIO_InitStruct);

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOD);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(LED_PIN_GPIO_Port, LED_PIN_Pin);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, BMP5_LED1_Pin|TEST_Pin);

  /**/
  LL_GPIO_ResetOutputPin(GPIOB, MS5525_LED_Pin|VALVE1_Pin|VALVE2_Pin|PUMP1_Pin
                          |PUMP2_Pin|BMP5_LED2_Pin|STEPPER_DIR_Pin);

  /**/
  LL_GPIO_SetOutputPin(NSPI_CS_GPIO_Port, NSPI_CS_Pin);

  /**/
  LL_GPIO_SetOutputPin(GPIOB, MS5525_POWER_Pin|BMP5_POWER_Pin);

  /**/
  GPIO_InitStruct.Pin = LED_PIN_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(LED_PIN_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BMP5_LED1_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(BMP5_LED1_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = NSPI_CS_Pin|TEST_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = MS5525_LED_Pin|BMP5_LED2_Pin|STEPPER_DIR_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = MS5525_POWER_Pin|BMP5_POWER_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = VALVE1_Pin|VALVE2_Pin|PUMP1_Pin|PUMP2_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = SW_UNITS_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(SW_UNITS_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
  /*
   * Check and reset the sensors if needed before I2C is initialized.
   */
    ms5525_check_reset();
    bmp580_check_reset();

/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

uint32_t check_reset_i2c_bus_intf(I2C_HandleTypeDef *hi2c)
{
  uint32_t sr1itflags = READ_REG(hi2c->Instance->SR1);
  uint32_t sr2itflags = READ_REG(hi2c->Instance->SR2);
  uint32_t error = 0;

  /* I2C Arbitration Lost error interrupt occurred ---------------------------*/
  if ((I2C_CHECK_FLAG(sr1itflags, I2C_FLAG_ARLO) != RESET))
  {
    error |= HAL_I2C_ERROR_ARLO;

    /* Clear ARLO flag */
    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_ARLO);
  }

  if ((I2C_CHECK_FLAG(sr1itflags, I2C_FLAG_BERR) != RESET) || (I2C_CHECK_FLAG(sr2itflags, I2C_FLAG_BUSY) != RESET))
  {
    error |= HAL_I2C_ERROR_BERR;
    __HAL_I2C_DISABLE(hi2c);

    /* Clear BERR flag */
    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_BERR);

    /* Save frequency and interrupt settings */
    uint32_t save_cr2 = hi2c->Instance->CR2;
    uint32_t save_ccr = hi2c->Instance->CCR;
    uint32_t save_trise = hi2c->Instance->TRISE;
    /* Workaround: Start cannot be generated after a misplaced Stop */
    SET_BIT(hi2c->Instance->CR1, I2C_CR1_SWRST);
    CLEAR_BIT(hi2c->Instance->CR1, I2C_CR1_SWRST);
    /* Restore */
    hi2c->Instance->CR2 = save_cr2;
    hi2c->Instance->CCR = save_ccr;
    hi2c->Instance->TRISE = save_trise;
    __HAL_I2C_ENABLE(hi2c);
  }

  return error;
}

void check_reset_i2c_bus(void)
{
  unsigned i;

  for (i = 0; i < lengthof(device_status_table); i++) {
      if (HAL_OK != device_status_table[i].status) {
	  check_reset_i2c_bus_intf(device_status_table[i].intf);
      }
  }
}

void read_sensors(void)
{
  if (NULL == ms5525_dev) {
	ms5525_dev = ms5525_setup(0, MS5525DSO_pp001DS, MS5525DSO_OSR_4096, device_status_table[DEV_SENSOR_MS5525].intf);
  } else {
	device_status_table[DEV_SENSOR_MS5525].status =
	    ms5525_read_data(ms5525_dev, &ms5525_data);
  }

  if (NULL == bmp5_dev1) {
	  bmp5_dev1 = bmp580_setup(0, device_status_table[DEV_SENSOR_BMP_1].intf);
  } else {
	device_status_table[DEV_SENSOR_BMP_1].status =
	    bmp580_read_measurements(&bmp5_data[0], bmp5_dev1);
  }

  if (NULL == bmp5_dev2) {
      bmp5_dev2 = bmp580_setup(1, device_status_table[DEV_SENSOR_BMP_1].intf);
  } else {
	device_status_table[DEV_SENSOR_BMP_2].status =
	    bmp580_read_measurements(&bmp5_data[1], bmp5_dev2);
  }

}

void update_indication_leds(void)
{
  sensors_active = HAL_OK;
  unsigned i;
  for (i = 0; i < lengthof(device_status_table); i++) {
      if (NULL != device_status_table[i].ind_port) {
	  if (HAL_OK == device_status_table[i].status) {
	      LL_GPIO_SetOutputPin(device_status_table[i].ind_port, device_status_table[i].ind_pin);
	  } else {
	      LL_GPIO_ResetOutputPin(device_status_table[i].ind_port, device_status_table[i].ind_pin);
	  }
      }
      if (HAL_OK != device_status_table[i].status) {
	  sensors_active = HAL_ERROR;
      }
  }
}

static uint32_t pa100_to_psi(uint32_t pa100)
{
  return pa100 / 6895;
}

static int32_t pa_to_psi(int32_t pa)
{
  return (pa * 100) / 6895;
}

static int32_t pa100_to_inhg(int32_t pa100)
{
  return pa100 / 3386;
}

static int32_t pa100_to_inh2o(int32_t pa100)
{
  return (pa100*10) / 2488;
}
/*
static int32_t pa_to_inhg(int32_t pa)
{
  return (pa * 100) / 3386;
}

static int32_t pa_to_inh2o(int32_t pa)
{
  return (pa*1000) / 2488;
}
*/
void update_screen(void)
{
  if (NULL == lcd_dev)
      return;

  uint32_t v1, v2;
  int32_t u1, u2;
  /* Use Pa */
  if (UNITS_SET1 == active_units) {
      v1 = bmp5_data[STATIC_PRESSURE_SENSOR_INDEX].pressure;
      v2 = serial_exchange_data.control.static_pressure*100;
      u1 = ms5525_data.pressure;
      u2 = serial_exchange_data.control.pitot_pressure*100;
  }

  /* Use PSI */
  if (UNITS_SET2 == active_units) {
      v1 = pa100_to_psi(bmp5_data[STATIC_PRESSURE_SENSOR_INDEX].pressure);
      v2 = pa_to_psi(serial_exchange_data.control.static_pressure);
      u1 = pa100_to_psi(ms5525_data.pressure);
      u2 = pa_to_psi(serial_exchange_data.control.pitot_pressure);
  }

  lcd_printf(lcd_dev, 0, 0, "%7u.%02u%6u.%02u", v1/100, v1%100, u1/100, u1%100);
  lcd_printf(lcd_dev, 0, 1, "%7u.%02u%6u.%02u", v2/100, v2%100, u2/100, u2%100);

  int32_t static_ambient_diff = (int32_t)bmp5_data[AMBIENT_PRESSURE_SENSOR_INDEX].pressure -
	  (int32_t)bmp5_data[STATIC_PRESSURE_SENSOR_INDEX].pressure;
  int32_t w1 = pa100_to_inhg(static_ambient_diff);
  int32_t w2 = pa100_to_inh2o(static_ambient_diff);
  lcd_printf(lcd_dev, 0, 2, "%7d.%02u%6d.%02u", w1/100, abs(w1%100), w2/100, abs(w2%100));

  int32_t pitot_static_diff = (int32_t)bmp5_data[AMBIENT_PRESSURE_SENSOR_INDEX].pressure -
	  (int32_t)ms5525_data.pressure;

  w1 = pa100_to_inhg(pitot_static_diff);
  w2 = pa100_to_inh2o(pitot_static_diff);

  lcd_printf(lcd_dev, 0, 3, "%7d.%02u%6d.%02u", w1/100, abs(w1%100), w2/100, abs(w2%100));

//  lcd_printf(lcd_dev, 0, 3, "T3=%3d.%02d\xb2", ms5525_data.temperature/100, ms5525_data.temperature%100);
  lcd_flush(lcd_dev);
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
