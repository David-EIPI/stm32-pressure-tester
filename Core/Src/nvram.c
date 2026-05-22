/*
 * nvram.c
 *
 *  Created on: May 1, 2026
 *      Author: shir
 */

#include "stm32f1xx_hal.h"
#include "string.h"
#include "main.h"

#include "nvram.h"

#define USE_HARDWARE_CRC_CALCULATOR 0
#define PAGE_SIZE 64
#define MAX_PAGE 16

static const unsigned DevAddr = 0xA0;

struct nvram_control_block_s {
  uint16_t serial_no;
  uint16_t version;
  uint32_t crc32;
};

struct nvram_page_s {
  uint8_t data[PAGE_SIZE - sizeof(struct nvram_control_block_s)];
  struct nvram_control_block_s ncb;
};

/* Points to the page where data can be stored when next write is requested. */
static uint8_t active_nvram_page = 0;
/* Points to the cached page that will be used for the next read or write request. */
static uint8_t active_cache_page = 0;
/* Flag indicates if the active cached page has valid data. */
static uint8_t valid_cache_page = 0;

/* NVRAM data cache */
static struct nvram_page_s page_cache[2] = {
    { .ncb = {0, 0} },
};


#if USE_HARDWARE_CRC_CALCULATOR
static uint32_t hw_crc32(void *data, size_t len)
{
  return HAL_CRC_Calculate(&hcrc, (uint32_t *)data, len/sizeof(uint32_t));
}

#define CALC_CRC32 hw_crc32

#else
#define CALC_CRC32 crc32_formula_reflect
/*
 * Formulaic CRC32 algorithm from: https://github.com/Michaelangel007/crc32
 * */
static uint32_t crc32_formula_reflect(const void *data, size_t len)
{
  const uint32_t POLY = 0xEDB88320;

  const unsigned char *buffer = (const unsigned char*) data;
  uint32_t crc = (uint32_t)-1;

  while( len-- ) {
      crc = crc ^ *buffer++;

      unsigned bit;
      for (bit = 0; bit < 8; bit++ ) {
          if (crc & 1 ) {
	      crc = (crc >> 1) ^ POLY;
          } else {
              crc = (crc >> 1);
          }
      }
  }
  return ~crc;
}
#endif
/*
 * Compare page data CRC32 with the stored CRC32.
 * Return 0 if CRC is correct.
 * */
static int check_page_crc(uint8_t cache_index)
{
  uint32_t crc = CALC_CRC32(&page_cache[cache_index].data, sizeof(page_cache[cache_index].data));
  return (int)(crc - page_cache[cache_index].ncb.crc32);
}

/*
 * Read page with the given index from the NVRAM device into the specified cache page.
 */
static HAL_StatusTypeDef read_page(I2C_HandleTypeDef *intf, uint8_t page_index, uint8_t cache_index)
{
  unsigned mem_addr = page_index;
  mem_addr *= PAGE_SIZE;
  HAL_StatusTypeDef rslt = HAL_I2C_Mem_Read(
        intf, 					// i2c handle
        DevAddr,				// i2c address, left aligned
    	mem_addr,				// register address
    	I2C_MEMADD_SIZE_16BIT,			// 24LC256 uses 16 bit memory addresses
    	(uint8_t *)&page_cache[cache_index],	// write returned data to this variable
    	sizeof(page_cache[0]),			// how many bytes to expect returned
    	50);					// timeout

  return rslt;
}

/*
 * Write the active cache page to the NVRAM device to the slot at the given index.
 */
static HAL_StatusTypeDef write_page(I2C_HandleTypeDef *intf, uint8_t page_index, uint8_t cache_index)
{
  unsigned mem_addr = page_index;
  mem_addr *= PAGE_SIZE;
  HAL_StatusTypeDef rslt = HAL_I2C_Mem_Write(
        intf, 					// i2c handle
        DevAddr,				// i2c address, left aligned
    	mem_addr,				// register address
    	I2C_MEMADD_SIZE_16BIT,			// 24LC256 uses 16 bit memory addresses
    	(uint8_t *)&page_cache[cache_index],	// write returned data to this variable
    	sizeof(page_cache[0]),			// how many bytes to expect returned
    	50);					// timeout

  return rslt;
}

/*
 * Read NVRAM pages until valid page with the most recent serial_no is found.
 * Pages with valid data are expected have sequential serial_no numbers.
 * Call to this functions updates page access variables.
 * Returns HAL_OK on success.
 */
HAL_StatusTypeDef nvram_setup(I2C_HandleTypeDef *intf, uint16_t version)
{
  unsigned i;
  unsigned ci = 0;
  int crc_diff = 0;

  active_nvram_page = 0;
  active_cache_page = ci;
  valid_cache_page = 0;

  HAL_StatusTypeDef rslt = read_page(intf, 0, ci);
  if (HAL_OK == rslt) {
      crc_diff = check_page_crc(ci);
      valid_cache_page = 0 == crc_diff && version == page_cache[ci].ncb.version;
  }

  for (i = 1; (i < MAX_PAGE)   &&
	      (HAL_OK == rslt) &&
	      (0 == crc_diff)  &&
	      version == page_cache[ci].ncb.version; i++)
  {
      typeof(page_cache[ci].ncb.serial_no) prev_serial_no = page_cache[ci].ncb.serial_no;

      if (++ci >= lengthof(page_cache))
	ci = 0;

      rslt = read_page(intf, i, ci);
      if (HAL_OK == rslt) {
          crc_diff = check_page_crc(ci);
	  if (0 == crc_diff && version == page_cache[ci].ncb.version) {
	      /* Check if serial_no numbers are increasing sequentially and stop if a gap is found */
	      if (page_cache[ci].ncb.serial_no != prev_serial_no + 1)
		break;

	      active_nvram_page = i;
	      active_cache_page = ci;
	  }
      }
  }

  if (HAL_OK == rslt && valid_cache_page) {
  /* Use next page for the next write request. */
      active_nvram_page += 1;
  }

  return rslt;
}

/*
 * Copy data from the cached page if it has valid data.
 * Return 0 on success.
 */
int nvram_read_data(I2C_HandleTypeDef *intf, void *data, uint32_t len, uint16_t version)
{
  if (valid_cache_page && version == page_cache[active_cache_page].ncb.version) {
      if (len >= sizeof(page_cache[active_cache_page].data))
	len = sizeof(page_cache[active_cache_page].data);

      memcpy(data, &page_cache[active_cache_page].data, len);
      return 0;
  }

  return -1;
}

/*
 * Copy data to the cached page, update serial_no and CRC32 and write the page
 * to the next slot on the NVRAM device.
 */
int nvram_write_data(I2C_HandleTypeDef *intf, const void *data, uint32_t len, uint16_t version)
{
  uint32_t serial_no = page_cache[active_cache_page].ncb.serial_no;

  if (valid_cache_page)
    serial_no += 1;
  else
    serial_no = 0;

  if (len >= sizeof(page_cache[active_cache_page].data))
	len = sizeof(page_cache[active_cache_page].data);

  memcpy(&page_cache[active_cache_page].data, data, len);
  valid_cache_page = 1;

  page_cache[active_cache_page].ncb.serial_no = serial_no;
  page_cache[active_cache_page].ncb.crc32 = CALC_CRC32(&page_cache[active_cache_page].data, sizeof(page_cache[active_cache_page].data));
  page_cache[active_cache_page].ncb.version = version;

  if (active_nvram_page >= MAX_PAGE)
    active_nvram_page = 0;

  if (HAL_OK == write_page(intf, active_nvram_page, active_cache_page)) {
      active_nvram_page += 1;
      return 0;
  }

  return -1;
}

