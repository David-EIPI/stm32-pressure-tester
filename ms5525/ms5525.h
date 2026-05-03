/*
 * ms5525.h
 *
 *  Created on: Mar 31, 2026
 *      Author: shir
 */

#ifndef MS5525_H_
#define MS5525_H_
#include <stdint.h>
#include <stddef.h>
#include "main.h"

typedef struct ms5525_dev *ms5525_ref;

/* *
 * Sensor variants for different pressure ranges
 */
typedef enum
{
  MS5525DSO_pp001DS = 0,
  MS5525DSO_pp002GS,
  MS5525DSO_pp002DS,
  MS5525DSO_pp005GS,
  MS5525DSO_pp005DS,
  MS5525DSO_pp015GS,
  MS5525DSO_pp015AS,
  MS5525DSO_pp015DS,
  MS5525DSO_pp030AS,
  MS5525DSO_pp030GS,
  MS5525DSO_pp030DS,

  MS5525DSO_pp_range_Count,
} MS5525DSO_range_t;

typedef enum
{
  MS5525DSO_OSR_256 = 0,
  MS5525DSO_OSR_512,
  MS5525DSO_OSR_1024,
  MS5525DSO_OSR_2048,
  MS5525DSO_OSR_4096,
  MS5525DSO_OSR_Count,
} MS5525DSO_OSR_t;

typedef enum
{
  MS5525DSO_DATA_NONE = 0,
  MS5525DSO_DATA_TEMPERATURE = 1,
  MS5525DSO_DATA_PRESSURE = 2,
  MS5525DSO_DATA_ALL = 3,
} MS5525DSO_DATA_TYPE_t;

typedef struct {
  int32_t temperature;
  int32_t pressure;
  MS5525DSO_DATA_TYPE_t type;
} ms5525dso_data_t;


ms5525_ref ms5525_setup(uint8_t sensor_idx, MS5525DSO_range_t range_var, MS5525DSO_OSR_t osr, I2C_HandleTypeDef *intf);
HAL_StatusTypeDef ms5525_read_data(ms5525_ref dev, ms5525dso_data_t *data);
HAL_StatusTypeDef ms5525_check_reset(void);
#endif /* MS5525_H_ */
