/*
 * nhd0420dzw.c
 *
 *  Created on: Apr 2, 2026
 *      Author: shir
 */

#include "main.h"
#include "nhd0420dzw.h"
#include "stdarg.h"
#include "stdio.h"
#include <float.h>

#define NLINE 4
#define NCOL 21

#define TEST_OUPUT_AFTER_SETUP 0

static struct nhd0420_dev {
  SPI_HandleTypeDef *intf;

  /*
   * This buffer stores text to be displayed in standard ASCII format.
   * Buffer holds 104 bytes covering 80 bytes of display DDRAM and
   * a 24 bytes gap between the lines.
   */
  uint32_t text_buf[26];

  /*
   * This buffer stores screen output data in the format needed by
   * SPI output routines (i.e. shifted right by 2 bits and prepended by RW/RS bits).
   */
  uint32_t screen_buf[27];

} nhd0420_dev = {
    .text_buf = { [0 ... 25] = 0x20202020 },
};


/*
 * Send up to 4 commands in one pack. If less are given, the last command is repeated.
 */
static
HAL_StatusTypeDef send_commands4(struct nhd0420_dev *dev, uint8_t *commands, unsigned count)
{
  uint32_t cmd5[2];

  uint8_t cmd = commands[0];
  cmd5[0] = ((cmd >> 2) <<  0) | ((cmd &  3) << 14); // 1st command

  if (count > 1)
    cmd = commands[1];

  cmd5[0] |= ((cmd >> 4) <<  8) | ((cmd & 15) << 20); // 2nd command

  if (count > 2)
    cmd = commands[2];

  cmd5[0] |= ((cmd >> 6) << 16) | ((cmd & 63) << 26);  // 3rd command

  if (count > 3)
    cmd = commands[2];

  cmd5[1] = commands[count - 1]; // 4th command
  LL_GPIO_ResetOutputPin(NSPI_CS_GPIO_Port, NSPI_CS_Pin);
  HAL_StatusTypeDef rslt = HAL_SPI_Transmit(dev->intf, (uint8_t *)&cmd5, 5, 100);
  LL_GPIO_SetOutputPin(NSPI_CS_GPIO_Port, NSPI_CS_Pin);
  return rslt;
}

/*
static
HAL_StatusTypeDef send_data(struct nhd0420_dev *dev, uint8_t *data, uint16_t count)
{
  uint8_t dbuf[count+1];
  uint16_t i;
  uint8_t upper2 = 0x2 << 6;
  for (i = 0; i < count; i++) {
      dbuf[i] = upper2 | (data[i] >> 2);
      upper2 = data[i] << 6;
  }
  dbuf[count] = upper2;
  HAL_StatusTypeDef rslt = HAL_SPI_Transmit(dev->intf, &dbuf[0], count+1, 100);
  __HAL_SPI_DISABLE(dev->intf);
  return rslt;
}
*/

static
void update_screen_buffer(struct nhd0420_dev *dev)
{
/* Begin with RW/RS bits for data output */
  uint32_t upper2 = 0x2 << 6;

  uint32_t i;
/*
 * Sequentially move bits from text buffer to screen buffer
 * shifting each byte right by 2 positions. To speed things up
 * 32 bit shifts are used. Since STM32 core is little-endian,
 * and display needs bits in big-endian order, bytes are swapped
 * between bit shifts.
 */
  for (i = 0; i < lengthof(dev->text_buf); i++) {
      uint32_t tmp = __builtin_bswap32(dev->text_buf[i]);
      dev->screen_buf[i] = upper2 + __builtin_bswap32(tmp >> 2);
      upper2 = (tmp & 3) << 6;
  }
  dev->screen_buf[lengthof(dev->text_buf)] = upper2;
}

static
HAL_StatusTypeDef send_buffer(struct nhd0420_dev *dev)
{
  LL_GPIO_ResetOutputPin(NSPI_CS_GPIO_Port, NSPI_CS_Pin);
  HAL_StatusTypeDef rslt = HAL_SPI_Transmit(dev->intf, (uint8_t*)&dev->screen_buf, 105, 100);
  LL_GPIO_SetOutputPin(NSPI_CS_GPIO_Port, NSPI_CS_Pin);
  return rslt;
}

static uint8_t setup_commands[] = {
     0x3b,     //function set
     0x06,     //entry mode set
     0x0c,     //display on
     0x02,     //return home
     0x01,     //clear display
};

static uint8_t home_command[] = {
    0x02, // return home
 //   0x80, // set DRAM address 0
};

static uint8_t buffer_offsets[NLINE] = {
    0, 64, 20, 84
};

int lcd_printf(nhd0420_ref dev, uint8_t x, uint8_t y, const char *fmt, ...)
{
  if (y >= NLINE) y = NLINE - 1;
  if (x >= NCOL) x = NCOL - 1;
  char *dstbuf = x + buffer_offsets[y] + (char *)&(dev->text_buf);

  va_list args;
  va_start(args, fmt);
  int rslt = vsnprintf(dstbuf, (size_t)(NCOL - x), fmt, args);
  va_end(args);
  if (x + rslt < NCOL)
    dstbuf[x+rslt]=' ';

  return rslt;
}

HAL_StatusTypeDef lcd_flush(nhd0420_ref dev)
{
  send_commands4(dev, home_command, 1);
  update_screen_buffer(dev);
  return send_buffer(dev);
}

#if TEST_OUPUT_AFTER_SETUP
static uint8_t teststr1[] = "0123456789abcdefghik";
static uint8_t teststr2[] = "lmnopqrstuvwxyz+-!@#";
static uint8_t teststr3[] = "0123456789ABCDEFGHIK";
static uint8_t teststr4[] = "LMNOPQRSTUVWXYZ$%^&*";
#endif

nhd0420_ref nhd0420_setup(SPI_HandleTypeDef *intf)
{
  nhd0420_dev.intf = intf;

  send_commands4(&nhd0420_dev, &setup_commands[0], 4);
  send_commands4(&nhd0420_dev, &setup_commands[4], 1);
  HAL_Delay(2); // additional delay after the Clear Display command

#if TEST_OUPUT_AFTER_SETUP
  lcd_printf(&nhd0420_dev, 0, 0, "%s", teststr1);
  lcd_printf(&nhd0420_dev, 0, 1, "%s", teststr2);
  lcd_printf(&nhd0420_dev, 0, 2, "%s", teststr3);
  lcd_printf(&nhd0420_dev, 0, 3, "%s", teststr4);
  int i;
  for (i = 0; i < 10; i++) {
    HAL_Delay(10000);
    lcd_flush(&nhd0420_dev);
  }
#endif
  return &nhd0420_dev;
}
