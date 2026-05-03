/*
 * nhd0420dzw.h
 *
 *  Created on: Apr 2, 2026
 *      Author: shir
 */

#ifndef NHD0420DZW_H_
#define NHD0420DZW_H_

typedef struct nhd0420_dev *nhd0420_ref;

/*
 * Setup initializes the screen and returns device handle needed
 * by the output functions.
 */
nhd0420_ref nhd0420_setup(SPI_HandleTypeDef *intf);

/*
 * Prints formatted text into the internal buffer.
 * Screen is not updated.
 */
int lcd_printf(nhd0420_ref dev, uint8_t x, uint8_t y, const char *fmt, ...);

/*
 * Updates the entire screen with the contents of the internal buffer.
 * At 2.25MHz sending the entire update command takes ~450us.
 * The SPI transaction is done in blocking mode.
 * This function returns immediately after the transaction but according
 * to the datasheet screen needs 600us to actually execute the command.
 */
HAL_StatusTypeDef lcd_flush(nhd0420_ref dev);

#endif /* NHD0420DZW_H_ */
