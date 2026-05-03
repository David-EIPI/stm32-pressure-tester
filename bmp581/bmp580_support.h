#ifndef _BMP580_SUPPORT_
#define _BMP580_SUPPORT_
#include "bmp5_defs.h"

typedef struct bmp5_dev *bmp580_ref;

/* Setup the BMP580 driver with the STM32-specific callbacks. */
bmp580_ref bmp580_setup(uint8_t sensor_idx, I2C_HandleTypeDef *intf);

/* Readout the data. In case of BMP580 humidity is not read. */
HAL_StatusTypeDef bmp580_read_measurements(struct bmp5_sensor_data *out_data, bmp580_ref bmp5_dev);

/* Implemented with SysTick, intf_ptr is ignored */
void BMP580_delay_us(uint32_t period, void *intf_ptr);

HAL_StatusTypeDef bmp580_check_reset(void);
#endif
