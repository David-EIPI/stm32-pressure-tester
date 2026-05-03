/*
 * nvram.h
 *
 *  Created on: May 1, 2026
 *      Author: shir
 */

#ifndef INC_NVRAM_H_
#define INC_NVRAM_H_


/*
 * Reads NVRAM and looks for valid saved data block. Must be called first
 * before calling the read and write functions.
 */
HAL_StatusTypeDef nvram_setup(I2C_HandleTypeDef *intf, uint16_t version);

/*
 * Copy data from the cached page if it has valid data.
 * Return 0 on success.
 */
int nvram_read_data(I2C_HandleTypeDef *intf, void *data, uint32_t len, uint16_t version);

/*
 * Copy data to the cached page, update serial_no and CRC32 and write the page
 * to the next slot on the NVRAM device.
 */
int nvram_write_data(I2C_HandleTypeDef *intf, const void *data, uint32_t len, uint16_t version);


#endif /* INC_NVRAM_H_ */
