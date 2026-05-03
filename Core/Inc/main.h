/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

#include "stm32f1xx_ll_tim.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_exti.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_cortex.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_pwr.h"
#include "stm32f1xx_ll_dma.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "bmp580_support.h"
#include "ms5525.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
enum device_status_e {
  DEV_SENSOR_BMP_1 = 0,
  DEV_SENSOR_BMP_2,
  DEV_SENSOR_MS5525,
  DEV_STATUS_COUNT,
};
/*
 * Device status indication data.
 * If defined, port and pin are used to indicate device status;
 */
typedef struct device_status_s {
  HAL_StatusTypeDef status;
  I2C_HandleTypeDef *intf;
  GPIO_TypeDef *ind_port;
  uint32_t ind_pin;
} device_status_t;

extern device_status_t device_status_table[DEV_STATUS_COUNT];

/*
 * Health status of all sensors.
 */
extern HAL_StatusTypeDef sensors_active;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
/*
 * 0 or 1 to choose which of the 2 BMP5 sensors to use as
 * a source of the static pressure.
 */
enum {
  AMBIENT_PRESSURE_SENSOR_INDEX = 0,
  STATIC_PRESSURE_SENSOR_INDEX = 1,
};

extern struct bmp5_sensor_data bmp5_data[2];
extern ms5525dso_data_t ms5525_data;

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define lengthof(a) (sizeof(a)/sizeof(a[0]))

#define CONCAT_HELPER(a, b) a ## b
#define CONCAT(a, b) CONCAT_HELPER(a, b)
#define StepperChannel CONCAT(LL_TIM_CHANNEL_CH, LL_STEPPER_CHANNEL)
#define LL_TIM_OC_SetCompareCH_Stepper CONCAT(LL_TIM_OC_SetCompareCH, LL_STEPPER_CHANNEL)
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
/**
 * TIM1 is setup to run at 1kHz, counting milliseconds.
 * This is 16-bit timer used to time short intervals needed by sensors.
 */
uint16_t ms_clock(void);
uint16_t us_clock(void);
void us_delay(uint32_t delay);


/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define StepperTimer TIM4
#define LL_STEPPER_CHANNEL 1
#define LED_PIN_Pin LL_GPIO_PIN_13
#define LED_PIN_GPIO_Port GPIOC
#define BMP5_LED1_Pin LL_GPIO_PIN_1
#define BMP5_LED1_GPIO_Port GPIOA
#define NSPI_CS_Pin LL_GPIO_PIN_4
#define NSPI_CS_GPIO_Port GPIOA
#define MS5525_LED_Pin LL_GPIO_PIN_0
#define MS5525_LED_GPIO_Port GPIOB
#define MS5525_POWER_Pin LL_GPIO_PIN_1
#define MS5525_POWER_GPIO_Port GPIOB
#define MS5525_SDA_Pin LL_GPIO_PIN_11
#define MS5525_SDA_GPIO_Port GPIOB
#define VALVE1_Pin LL_GPIO_PIN_12
#define VALVE1_GPIO_Port GPIOB
#define VALVE2_Pin LL_GPIO_PIN_13
#define VALVE2_GPIO_Port GPIOB
#define PUMP1_Pin LL_GPIO_PIN_14
#define PUMP1_GPIO_Port GPIOB
#define PUMP2_Pin LL_GPIO_PIN_15
#define PUMP2_GPIO_Port GPIOB
#define TEST_Pin LL_GPIO_PIN_10
#define TEST_GPIO_Port GPIOA
#define BMP5_LED2_Pin LL_GPIO_PIN_3
#define BMP5_LED2_GPIO_Port GPIOB
#define SW_UNITS_Pin LL_GPIO_PIN_4
#define SW_UNITS_GPIO_Port GPIOB
#define STEPPER_DIR_Pin LL_GPIO_PIN_5
#define STEPPER_DIR_GPIO_Port GPIOB
#define STEPPER_STEP_Pin LL_GPIO_PIN_6
#define STEPPER_STEP_GPIO_Port GPIOB
#define BMP5_POWER_Pin LL_GPIO_PIN_7
#define BMP5_POWER_GPIO_Port GPIOB
#define BMP5_SDA_Pin LL_GPIO_PIN_9
#define BMP5_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
