/*
 * serial.h
 *
 *  Created on: Apr 4, 2026
 *      Author: shir
 */

#ifndef INC_SERIAL_H_
#define INC_SERIAL_H_

#define SERIAL_RX_BUFFER_SIZE 256

/*
 * This version is used when storing/loading the control data to/from NVRAM.
 * If data structure changes version number should change too.
 * This macro is a simple combination of the serial index and data size.
 * */
#define CONTROL_DATA_VERSION ((1 << 8) + sizeof(serial_exchange_data.control))

/*
 * Data received and transmitted via serial line.
 * This includes user settings, circular buffer, command mailboxes, etc.
 */

struct rx_buffer_data_s {
    uint8_t buffer[SERIAL_RX_BUFFER_SIZE];
    volatile int16_t head;
    volatile int16_t tail;
};

typedef struct serial_exchange_data_s {
/*
 * User data
 * */
  struct {
    int32_t static_pressure;
    int32_t pitot_pressure;
    int32_t tolerance_s;
    int32_t tolerance_p;
    int32_t stepper_freq; /* Hz */
    int32_t stepper_p;    /* P coefficient, 1/256 x Hz/Pa */
    unsigned sm_debug_output:2;
    unsigned sm_modified:1;
  } control;
/*
 * Ring buffer for input stream
 */
  struct rx_buffer_data_s rx;
} serial_exchange_data_t;


extern serial_exchange_data_t serial_exchange_data;

/*
 * Helper function to print formatted text to the USB/serial port.
 */
int usb_cdc_printf(const char *fmt, ...);

/*
 * USB virtual COM port input handler.
 * Called internally from the middleware USB processing code.
 */
void USB_CDC_RxHandler(uint8_t*, uint32_t);

/*
 * User input processing routine.
 * Should be called from the main loop.
 * Input stream is parsed and detected commands
 * are dispatched to the matching handlers.
 */
void process_serial_input(void);


/* *
 * Helper functions to handle parameters in NVRAM
 */

/*
 * Called once after boot to find and load previously saved parameters.
 */
void load_control_parameters(I2C_HandleTypeDef *intf);

/*
 * Called periodically to check if any control parameter was modified
 * and if yes save the updated parameters.
 */
void check_save_control_parameters(I2C_HandleTypeDef *intf);



#endif /* INC_SERIAL_H_ */
