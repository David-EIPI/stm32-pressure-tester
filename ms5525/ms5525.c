/*
 * ms5525.c
 *
 *  Created on: Mar 31, 2026
 *      Author: shir
 */


#include "main.h"
#include "ms5525.h"

#define MS5525DSO_ADDRESS1 (0x77<<1)
#define MS5525DSO_ADDRESS2 (0x76<<1)


#define MS5525DSO_CMD_CONVERT       0x40
#define MS5525DSO_CMD_RESET         0x1E
#define MS5525DSO_CMD_BASE_PROM     0xA0
#define MS5525DSO_CMD_CONVERT_D1    0x40
#define MS5525DSO_CMD_CONVERT_D2    0x50
#define MS5525DSO_CMD_READ_ADC      0x00

#ifndef lengthof
#define lengthof(a) (sizeof(a)/sizeof(a[0]))
#endif

typedef int8_t Q_coeff_t[6];

enum ms5525dso_dev_status_t {
  MS5525DSO_DEV_UNKNOWN = 0,
  MS5525DSO_DEV_RESET,
  MS5525DSO_DEV_HAVE_CONFIG,
};

enum ms5525dso_measurement_state_t {
  MS5525DSO_MEASURE_IDLE = 0,
  MS5525DSO_MEASURE_BUSY_T,
  MS5525DSO_MEASURE_DONE_T,
  MS5525DSO_MEASURE_BUSY_P,
  MS5525DSO_MEASURE_DONE_P,
};


const Q_coeff_t Q_coeff[MS5525DSO_pp_range_Count] =
{
  { 15, 17, 7, 5, 7, 21 }, // pp001DS
  { 14, 16, 8, 6, 7, 22 }, // pp002GS
  { 16, 18, 6, 4, 7, 22 }, // pp002DS
  { 16, 17, 6, 5, 7, 21 }, // pp005GS
  { 17, 19, 5, 3, 7, 22 }, // pp005DS
  { 16, 17, 6, 5, 7, 22 }, // pp015GS
  { 16, 17, 6, 5, 7, 22 }, // pp015AS
  { 17, 19, 5, 3, 7, 22 }, // pp015DS
  { 17, 18, 5, 4, 7, 22 }, // pp030AS
  { 17, 18, 5, 4, 7, 22 }, // pp030GS
  { 18, 21, 4, 1, 7, 22 }, // pp030DS
};

uint16_t measurement_time_ms[MS5525DSO_OSR_Count] = {
    1, 2, 3, 5, 10,
};


struct ms5525_dev {
  const Q_coeff_t* Q_ref;
  uint16_t C[7];
  uint8_t status;
  uint8_t addr;
  I2C_HandleTypeDef *intf;
  uint16_t start_time;
  MS5525DSO_OSR_t osr:8;
  enum ms5525dso_measurement_state_t m_state:8;
  int32_t raw_t;
  int32_t diff_t;
  int32_t raw_p;
  ms5525dso_data_t data;
} ms5525dso[2] = {
    {
	.addr = MS5525DSO_ADDRESS1,
	.status = MS5525DSO_DEV_UNKNOWN,
	.m_state = MS5525DSO_MEASURE_IDLE,
    },
    {
	.addr = MS5525DSO_ADDRESS2,
	.status = MS5525DSO_DEV_UNKNOWN,
	.m_state = MS5525DSO_MEASURE_IDLE,
    },
};

HAL_StatusTypeDef ms5525_send_cmd(ms5525_ref dev, uint8_t cmd)
{
  HAL_StatusTypeDef status = HAL_OK;

#if 0
  status = HAL_I2C_Mem_Write(dev->intf,	// i2c handle
        dev->addr,	                // i2c address, left aligned
	(uint8_t)cmd,			// register address
	I2C_MEMADD_SIZE_8BIT,		// ms5525 uses 8bit commands
	NULL,				// no command data
	0,				// no command data
	50);				// timeout
#else
  status = HAL_I2C_Master_Transmit(dev->intf, dev->addr, &cmd, 1, 50);
#endif
	return status;
}

HAL_StatusTypeDef ms5525_read_mem(ms5525_ref dev, uint8_t addr, void *data, uint8_t len)
{
  HAL_StatusTypeDef rslt = HAL_OK;
#if 0
  rslt = HAL_I2C_Mem_Read(dev->intf,		// i2c handle
        dev->addr,				// i2c address, left aligned
	addr,					// register address
	I2C_MEMADD_SIZE_8BIT,			// ms5255 uses 8bit register addresses
	data,					// write returned data to this variable
	len,					// how many bytes to expect returned
	50);					// timeout
#else

  rslt = HAL_I2C_Master_Transmit(dev->intf, dev->addr, &addr, 1, 50);
  if (HAL_OK == rslt) {
      us_delay(50);
      rslt = HAL_I2C_Master_Receive(dev->intf, dev->addr, data, len, 50 );
  }
#endif
  return rslt;
}

HAL_StatusTypeDef ms5525_read_coeffs(ms5525_ref dev)
{
  unsigned i;
  HAL_StatusTypeDef rslt = HAL_OK;

  for (i = 0; i < lengthof(dev->C) && HAL_OK == rslt; i++) {
      rslt = ms5525_read_mem(dev, MS5525DSO_CMD_BASE_PROM + i * 2, &dev->C[i], sizeof(dev->C[i]));
      dev->C[i] = __builtin_bswap16(dev->C[i]);
  }
  return rslt;
}

/**
 * @brief Sensor initialization. Verifies sensor presence, reads C coefficient table.
 * @param sensor_idx I2C address bit: 0 or 1.
 * @param range_var Sensor range variant from the table. It can not be determined from the sensor itself, and must be provided here.
 * @param intf STM32 I2C interface handle.
 */
ms5525_ref ms5525_setup(uint8_t sensor_idx, MS5525DSO_range_t range_var,  MS5525DSO_OSR_t osr, I2C_HandleTypeDef *intf)
{
    sensor_idx &= 1; /* safety check, only 1 address bit is supported */
    ms5525_ref dev = &ms5525dso[sensor_idx];
    dev->intf = intf;
    dev->osr = osr;

    /* Select Q coefficients table for the given range */
    if (range_var > lengthof(Q_coeff))
	range_var = 0;

    dev->Q_ref = &Q_coeff[range_var];

    HAL_StatusTypeDef rslt = HAL_OK;

    /* Issue reset command as recommended in the datasheet */
    if (MS5525DSO_DEV_RESET != dev->status) {
	rslt = ms5525_send_cmd(dev, MS5525DSO_CMD_RESET);
	HAL_Delay(4);
    }
    if (HAL_OK == rslt) {
	dev->status = MS5525DSO_DEV_RESET;
    }

    if (MS5525DSO_DEV_RESET == dev->status) {
	rslt = ms5525_read_coeffs(dev);
    }

    if (HAL_OK == rslt) {
	dev->status = MS5525DSO_DEV_HAVE_CONFIG;
	return dev;
    }

    return NULL;
}

static HAL_StatusTypeDef ms5525_convert_d1(ms5525_ref dev)
{
  HAL_StatusTypeDef rslt = ms5525_send_cmd(dev, MS5525DSO_CMD_CONVERT_D1 + dev->osr * 2);
  if (HAL_OK == rslt) {
      dev->m_state = MS5525DSO_MEASURE_BUSY_P;
      dev->start_time = ms_clock();
  }
  return rslt;
}

static HAL_StatusTypeDef ms5525_convert_d2(ms5525_ref dev)
{
  HAL_StatusTypeDef rslt = ms5525_send_cmd(dev, MS5525DSO_CMD_CONVERT_D2 + dev->osr * 2);
  if (HAL_OK == rslt) {
      dev->m_state = MS5525DSO_MEASURE_BUSY_T;
      dev->start_time = ms_clock();
  }
  return rslt;
}

static HAL_StatusTypeDef ms5525_read_adc(ms5525_ref dev, void *data)
{
  HAL_StatusTypeDef rslt = ms5525_read_mem(dev, MS5525DSO_CMD_READ_ADC, data, 3);
  if (HAL_OK == rslt) {
      dev->m_state += 1;
  }
  return rslt;
}

static void ms5525_calculate(ms5525_ref dev)
{
  if (MS5525DSO_MEASURE_DONE_T == dev->m_state) {
      int32_t raw_t = (int32_t)(__builtin_bswap32(dev->raw_t) >> 8);
      const int8_t *Q = *(dev->Q_ref) - 1;
      dev->diff_t = raw_t - (dev->C[5] << Q[5]);
      dev->data.temperature = 2000 + (int32_t)(((int64_t)dev->diff_t * dev->C[6]) >> Q[6]);
      dev->data.type |= MS5525DSO_DATA_TEMPERATURE;
  }
  if (MS5525DSO_MEASURE_DONE_P == dev->m_state) {
      int32_t raw_p = (int32_t)(__builtin_bswap32(dev->raw_p) >> 8);
      const int8_t *Q = *(dev->Q_ref) - 1;
      int64_t off = ((int64_t)dev->C[2] << Q[2]) + (((int64_t)dev->C[4] * dev->diff_t) >> Q[4]);
      int64_t sens = ((int64_t)dev->C[1] << Q[1]) + (((int64_t)dev->C[3] * dev->diff_t) >> Q[3]);
      int32_t p_psi = (int32_t)((((sens * raw_p) >> 21) - off) >> 15);
      dev->data.pressure = (p_psi * 6895) / 10000; /* convert to Pa */
      dev->data.type |= MS5525DSO_DATA_PRESSURE;
  }
}

HAL_StatusTypeDef ms5525_start_measurement(ms5525_ref dev)
{
  if (MS5525DSO_DEV_HAVE_CONFIG != dev->status) {
  	return HAL_ERROR;
  }

  enum ms5525dso_measurement_state_t state = dev->m_state;

  if (MS5525DSO_MEASURE_IDLE != state)
    return HAL_ERROR;

  dev->data.type = MS5525DSO_DATA_NONE;
  HAL_StatusTypeDef rslt = ms5525_convert_d2(dev);
  if (HAL_OK == rslt) {
      dev->m_state = MS5525DSO_MEASURE_BUSY_T;
  }
  return rslt;
}


HAL_StatusTypeDef ms5525_read_data(ms5525_ref dev, ms5525dso_data_t *data)
{
  if (MS5525DSO_DEV_HAVE_CONFIG != dev->status) {
  	return HAL_ERROR;
  }

  enum ms5525dso_measurement_state_t state = dev->m_state;
  uint16_t ms = ms_clock();
  uint16_t dt;
  HAL_StatusTypeDef rslt = HAL_OK;

  switch (state) {
    case MS5525DSO_MEASURE_IDLE:
      dev->data.type = MS5525DSO_DATA_NONE;
      *data = dev->data;
      ms5525_start_measurement(dev);
      break;
    case MS5525DSO_MEASURE_BUSY_T:
      dt = ms - dev->start_time;
      if (dt > measurement_time_ms[dev->osr] + 1) {
	  rslt = ms5525_read_adc(dev, &dev->raw_t);
      }
      break;
    case MS5525DSO_MEASURE_DONE_T:
      ms5525_calculate(dev);
      *data = dev->data;
      rslt = ms5525_convert_d1(dev);
      break;
    case MS5525DSO_MEASURE_BUSY_P:
      dt = ms - dev->start_time;
      if (dt > measurement_time_ms[dev->osr] + 1) {
	  rslt = ms5525_read_adc(dev, &dev->raw_p);
      }
      break;
    case MS5525DSO_MEASURE_DONE_P:
      ms5525_calculate(dev);
      *data = dev->data;
      dev->m_state = MS5525DSO_MEASURE_IDLE;
      break;
    default:
      break;
  }


  return rslt;
}

HAL_StatusTypeDef ms5525_check_reset(void)
{

  LL_GPIO_SetPinMode(MS5525_SDA_GPIO_Port, MS5525_SDA_Pin, LL_GPIO_MODE_FLOATING);
/*
 * Attempt to reset sensor from blocking SDA.
 */
  int i;
  for (i = 0; i < 5 && !LL_GPIO_IsInputPinSet(MS5525_SDA_GPIO_Port, MS5525_SDA_Pin); i++) {
      LL_GPIO_ResetOutputPin(MS5525_POWER_GPIO_Port, MS5525_POWER_Pin);
      HAL_Delay(500);
      LL_GPIO_SetOutputPin(MS5525_POWER_GPIO_Port, MS5525_POWER_Pin);
      HAL_Delay(500);
  }

  HAL_StatusTypeDef rslt = LL_GPIO_IsInputPinSet(MS5525_SDA_GPIO_Port, MS5525_SDA_Pin) ? HAL_OK : HAL_ERROR;

  LL_GPIO_SetPinMode(MS5525_SDA_GPIO_Port, MS5525_SDA_Pin, LL_GPIO_MODE_ALTERNATE);
  return rslt;
}

