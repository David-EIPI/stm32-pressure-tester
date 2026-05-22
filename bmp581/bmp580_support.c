/*
****************************************************************************
* bmp580_support.c
* Date: 2016/07/04
* Revision: 1.0.0
* Modified from (below):
*
* Copyright (C) 2015 - 2016 Bosch Sensortec GmbH
* bme280_support.c
* Date: 2016/07/04
* Revision: 1.0.6 $
*
* Usage: Sensor Driver support file for BME280 sensor
*
****************************************************************************
* License:
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*   Redistributions of source code must retain the above copyright
*   notice, this list of conditions and the following disclaimer.
*
*   Redistributions in binary form must reproduce the above copyright
*   notice, this list of conditions and the following disclaimer in the
*   documentation and/or other materials provided with the distribution.
*
*   Neither the name of the copyright holder nor the names of the
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
* OR CONTRIBUTORS BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
* OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
* PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
*
* The information provided is believed to be accurate and reliable.
* The copyright holder assumes no responsibility
* for the consequences of use
* of such information nor for any infringement of patents or
* other rights of third parties which may result from its use.
* No license is granted by implication or otherwise under any patent or
* patent rights of the copyright holder.
**************************************************************************/
/*---------------------------------------------------------------------------*/
/* Includes*/
/*---------------------------------------------------------------------------*/

#include "main.h"
#include "bmp580_support.h"
#include "bmp5.h"

/* STM HAL routines expect 7-bit address in the upper bits */
#define BMP580_ADDRESS1 (0x46 << 1)
#define BMP580_ADDRESS2 (0x47 << 1)

int8_t BMP580_I2C_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);

int8_t BMP580_I2C_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);

/********************End of I2C/SPI function declarations***********************/


/*	Brief : The delay routine
 *	\param : delay in us
*/
void BMP580_delay_us(uint32_t period, void *intf_ptr);

static struct bmp580_intf_desc {
    /*! I2C address
     * Interface will try all addresses and
     * store the first one detected
     */
    uint8_t addr;

    /*
     *	Target interface handle
     * */
    I2C_HandleTypeDef *intf;
} bmp580_intf_desc[2] = {
    {
		.addr = BMP580_ADDRESS1,
		.intf = NULL,
    },
    {
		.addr = BMP580_ADDRESS2,
		.intf = NULL,
    },
};

/*----------------------------------------------------------------------------*
 *  struct bmp580_t parameters can be accessed by using bmp580
 *	bmp580_t having the following parameters
 *	Bus write function pointer: BMP580_WR_FUNC_PTR
 *	Bus read function pointer: BMP580_RD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *	I2C address: dev_addr
 *	Interface of the sensor: .intf
 *---------------------------------------------------------------------------*/
static struct bmp5_dev bmp580[2]= {
    {
	.intf = BMP5_I2C_INTF,
	.write = BMP580_I2C_write,
	.read = BMP580_I2C_read,
	.delay_us = BMP580_delay_us,
	.intf_ptr = &bmp580_intf_desc[0],
    },
    {
	.intf = BMP5_I2C_INTF,
	.write = BMP580_I2C_write,
	.read = BMP580_I2C_read,
	.delay_us = BMP580_delay_us,
	.intf_ptr = &bmp580_intf_desc[1],
    },
};

static struct bmp5_osr_odr_press_config settings = {0};

/*
 * Precomputed systicks per 1 microsecond
 * */
uint32_t SysTicks_us = 0;

/*	Brief : The delay routine
 *	\param : delay in us
*/
void BMP580_delay_us(uint32_t period, void *intf_ptr)
{
	if (period > 1000) {
		HAL_Delay(period / 1000);
		period %= 1000;
	}
	uint32_t loop_usec = 500; /* Should be smaller than SysTick reload value, usually =1000us */
	do {
		if (period < loop_usec) {
			loop_usec = period;
		}
		period -= loop_usec;
		uint32_t start = SysTick->VAL;
		uint32_t ticks = loop_usec * SysTicks_us;
		while (start - SysTick->VAL < ticks) ;
	} while (period > 0);
}

/************** I2C/SPI buffer length ******/

#define	I2C_BUFFER_LEN 28


 /*	\Brief: The function is used as I2C bus write
 *	\Return : Status of the I2C write
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, will data is going to be written
 *	\param reg_data : It is a value hold in the array,
 *		will be used for write the value into the register
 *	\param cnt : The no of byte of data to be write
 */
int8_t BMP580_I2C_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    HAL_StatusTypeDef status = HAL_OK;
    int32_t iError = BMP5_OK;
    struct bmp580_intf_desc *intf = (struct bmp580_intf_desc *)intf_ptr;
    I2C_HandleTypeDef *hi2c = intf->intf;

    status = HAL_I2C_Mem_Write(hi2c,	// i2c handle
        intf->addr,	                // i2c address, left aligned
	(uint8_t)reg_addr,		// register address
	I2C_MEMADD_SIZE_8BIT,		// bmp580 uses 8bit register addresses
	(uint8_t*)(reg_data),		// write data provided in reg_data
	len,				// write how many bytes
	100);				// timeout

	if (status != HAL_OK)
    {
        // The BMP580 API calls for 0 return value as a success, and <0 returned as failure
    	iError = (BMP5_E_COM_FAIL);
    }
	return (int8_t)iError;
}

 /*	\Brief: The function is used as I2C bus read
 *	\Return : Status of the I2C read
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, will data is going to be read
 *	\param reg_data : This data read from the sensor, which is hold in an array
 *	\param cnt : The no of data byte of to be read
 */
int8_t BMP580_I2C_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    HAL_StatusTypeDef status = HAL_OK;
    int32_t iError = BMP5_OK;
    struct bmp580_intf_desc *intf = (struct bmp580_intf_desc *)intf_ptr;
    I2C_HandleTypeDef *hi2c = intf->intf;

    status = HAL_I2C_Mem_Read(hi2c,		// i2c handle
        intf->addr,				// i2c address, left aligned
	(uint8_t)reg_addr,			// register address
	I2C_MEMADD_SIZE_8BIT,			// bmp580 uses 8bit register addresses
	(uint8_t*)(reg_data),			// write returned data to this variable
	len,					// how many bytes to expect returned
	50);					// timeout

    if (status != HAL_OK)
    {
    	// The BMP580 API calls for 0 return value as a success, and <0 returned as failure
    	iError = (BMP5_E_COM_FAIL);
    }

    return (int8_t)iError;
}


/*--------------------------------------------------------------------------*
*	Initialize, reset and update the settings.
*	Up to 2 sensors can share 1 I2C bus.
*	Parameter sensor_idx must be 0 or 1, choosing which address to use.
*-------------------------------------------------------------------------*/

bmp580_ref bmp580_setup(uint8_t sensor_idx, I2C_HandleTypeDef *intf)
{
    int8_t rslt;
    SysTicks_us = SystemCoreClock / 1000000U;
    uint8_t sel_dev = sensor_idx;

    ((struct bmp580_intf_desc *)bmp580[sel_dev].intf_ptr)->intf = intf;
    rslt = bmp5_init(&bmp580[sel_dev]);

    if (rslt != BMP5_OK) {
	return NULL;
    }

    rslt = bmp5_get_osr_odr_press_config(&settings, &bmp580[sel_dev]);
    if (rslt != BMP5_OK) {
	return NULL;
    }

    /* Configuring the over-sampling rate, odr rate and enable pressure output */
    /* Overwrite the desired settings */
    /* Need to enable pressure reading */
    settings.press_en = BMP5_ENABLE;

    /* Over-sampling rate for temperature and pressure, */
    settings.osr_p = BMP5_OVERSAMPLING_128X;
    settings.osr_t = BMP5_OVERSAMPLING_64X;

    /* Select highest available data rate - must match the OSR settings (data sheet Table 7). */
    settings.odr = BMP5_ODR_10_HZ;

    rslt = bmp5_set_osr_odr_press_config(&settings, &bmp580[sel_dev]);
    if (rslt != BMP5_OK) {
	return NULL;
    }

    /* Always set the power mode after setting the configuration */
    rslt = bmp5_set_power_mode(BMP5_POWERMODE_CONTINOUS, &bmp580[sel_dev]);
    if (rslt != BMP5_OK) {
        return NULL;
    }

    return &bmp580[sel_dev];
}

HAL_StatusTypeDef bmp580_read_measurements(struct bmp5_sensor_data *out_data, bmp580_ref bmp5_dev)
{
    int rslt;

    rslt = bmp5_get_sensor_data(out_data, &settings, bmp5_dev);
    if (rslt != BMP5_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef bmp580_check_reset(void)
{
  LL_GPIO_SetPinMode(BMP5_SDA_GPIO_Port, BMP5_SDA_Pin, LL_GPIO_MODE_FLOATING);
/*
 * Attempt to reset sensor from blocking SDA.
 */
  int i;
  for (i = 0; i < 5 && !LL_GPIO_IsInputPinSet(BMP5_SDA_GPIO_Port, BMP5_SDA_Pin); i++) {
      LL_GPIO_ResetOutputPin(BMP5_POWER_GPIO_Port, BMP5_POWER_Pin);
      HAL_Delay(500);
      LL_GPIO_SetOutputPin(BMP5_POWER_GPIO_Port, BMP5_POWER_Pin);
      HAL_Delay(500);
  }

  HAL_StatusTypeDef rslt = LL_GPIO_IsInputPinSet(BMP5_SDA_GPIO_Port, BMP5_SDA_Pin) ? HAL_OK : HAL_ERROR;

  LL_GPIO_SetPinMode(BMP5_SDA_GPIO_Port, BMP5_SDA_Pin, LL_GPIO_MODE_ALTERNATE);
  return rslt;
}

