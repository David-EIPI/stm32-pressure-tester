/*
 * serial.c
 *
 *  Created on: Apr 4, 2026
 *      Author: shir
 *
 * The code in this file handles user communications.
 * It reads and parses user input, prints data on request,
 * receives adjustable parameters, stores parameters in NVRAM.
 *
 */

#include <stdint.h>
#include "stdarg.h"
#include "main.h"
#include "usbd_cdc_if.h"
#include "serial.h"
#include "bmp580_support.h"
#include "ms5525.h"
#include "nvram.h"

serial_exchange_data_t serial_exchange_data = {
    .rx = {
	.head = 0,
	.tail = 0,
    },
    .control.static_pressure = 100000,
    .control.pitot_pressure = 1000,
    .control.tolerance_s = 1000,
    .control.tolerance_p = 100,
    .control.stepper_freq = 1000,
    .control.stepper_p = 10,
};

//extern struct bmp5_sensor_data bmp5_data[2];
//extern ms5525dso_data_t ms5525_data;

#define MISSING_PARAM 0x80000000
#define CDC_OUTPUT_DELAY 10 /* milliseconds */

#define MIN_STATIC_TOLERANCE_PA 1
#define MIN_PITOT_TOLERANCE_PA  1

struct rx_parser_s {
    uint8_t state;

    uint8_t crc_present;
    uint8_t crc_expected;
    uint8_t crc;

    uint8_t code;
    uint8_t sign;
    uint8_t digits;
    uint8_t got_sign;

    uint8_t crc_digit_count;
    uint32_t value;
};

/* Buffer for use with sprintf()/CDC_Transmit_FS() */
static uint8_t cdc_buf[256];
extern USBD_HandleTypeDef hUsbDeviceFS;

/*
 * Helper function to print formatted text to the USB/serial port.
 */
int usb_cdc_printf(const char *fmt, ...)
{
/* Wait until previous transmit request completes. */
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;

  /* Limited wait time to avoid blocking if USB port becomes unusable */
  int i = 100;
  while (hcdc->TxState != 0 && i--) {
      us_delay(10);
  }
  va_list args;
  va_start(args, fmt);
  int rslt = vsnprintf((char *)cdc_buf, sizeof(cdc_buf), fmt, args);
  va_end(args);
  if (rslt > sizeof(cdc_buf))
    rslt = sizeof(cdc_buf);
  if (rslt > 0)
    CDC_Transmit_FS(cdc_buf, rslt);

  return rslt;
}

/*
 * Forward declaration for the exported helper function - process_serial_input.
 * */
static struct rx_parser_s rx_parser_state;

static inline uint16_t rx_buffer_len(const struct rx_buffer_data_s *b);
static inline uint8_t rx_buffer_put(struct rx_buffer_data_s *b, uint8_t c);
static inline uint8_t rx_buffer_get(struct rx_buffer_data_s *b, uint8_t *c);
static void rx_parse_buffer(struct rx_buffer_data_s *rb, struct rx_parser_s *p);

enum _nvram_status_code {
  NVRAM_STATUS_READY = 0,
  NVRAM_STATUS_ERROR = 1,
  NVRAM_STATUS_UNKNOWN = 2,
};

static enum _nvram_status_code nvram_status = NVRAM_STATUS_UNKNOWN;

/*
 * Callback that is invoked on user input.
 * Stores received data in the ring buffer.
 */
void USB_CDC_RxHandler(uint8_t* Buf, uint32_t Len)
{
  uint32_t i;
  for (i = 0; i < Len; i++) {
      if (! rx_buffer_put(&serial_exchange_data.rx, Buf[i]))
	break;
  }
}

/*
 * Exported helper function to be called from the main loop.
 */
void process_serial_input(void)
{
    rx_parse_buffer(&serial_exchange_data.rx, &rx_parser_state);
}

/* *
 * Helper functions to handle parameters in NVRAM
 */

/*
 * Called once after boot to find and load previously saved parameters.
 */
void load_control_parameters(I2C_HandleTypeDef *intf)
{
  HAL_StatusTypeDef rslt = nvram_setup(intf, CONTROL_DATA_VERSION);
  if (HAL_OK == rslt) {
      nvram_status = NVRAM_STATUS_READY;
      if (0 == nvram_read_data(intf, &serial_exchange_data.control, sizeof(serial_exchange_data.control), CONTROL_DATA_VERSION)) {
	  serial_exchange_data.control.sm_modified = 0;
	  serial_exchange_data.control.sm_debug_output = 0;
      }
  }
}

/*
 * Called periodically to check if any control parameter was modified
 * and if yes save the updated parameters.
 */
void check_save_control_parameters(I2C_HandleTypeDef *intf)
{
  if (NVRAM_STATUS_READY == nvram_status) {
      if (serial_exchange_data.control.sm_modified) {
	  if (0 == nvram_write_data(intf, &serial_exchange_data.control, sizeof(serial_exchange_data.control), CONTROL_DATA_VERSION)) {
	      serial_exchange_data.control.sm_modified = 0;
	  }
      }
  }
}


/*
 * Command handlers
 */

static
void rx_handler_static_pressure(uint8_t code, int32_t parameter)
{
  if (parameter != MISSING_PARAM) {
      serial_exchange_data.control.sm_modified = serial_exchange_data.control.static_pressure != parameter;
      serial_exchange_data.control.static_pressure = parameter;
  }
}

static
void rx_handler_pitot_pressure(uint8_t code, int32_t parameter)
{
  if (parameter != MISSING_PARAM) {
      serial_exchange_data.control.sm_modified = serial_exchange_data.control.pitot_pressure != parameter;
      serial_exchange_data.control.pitot_pressure = parameter;
  }
}

static
void rx_handler_pressure_tolerance_s(uint8_t code, int32_t parameter)
{
  if (parameter != MISSING_PARAM && parameter >= MIN_STATIC_TOLERANCE_PA) {
      serial_exchange_data.control.sm_modified = serial_exchange_data.control.tolerance_s != parameter;
      serial_exchange_data.control.tolerance_s = parameter;
  }
}

static
void rx_handler_pressure_tolerance_p(uint8_t code, int32_t parameter)
{
  if (parameter != MISSING_PARAM && parameter >= MIN_PITOT_TOLERANCE_PA) {
      serial_exchange_data.control.sm_modified = serial_exchange_data.control.tolerance_p != parameter;
      serial_exchange_data.control.tolerance_p = parameter;
  }
}

static
void rx_handler_stepper_freq(uint8_t code, int32_t parameter)
{
  if (parameter != MISSING_PARAM) {
      serial_exchange_data.control.sm_modified = serial_exchange_data.control.stepper_freq != parameter;
      serial_exchange_data.control.stepper_freq = parameter;
  }
}

static
void rx_handler_stepper_coef(uint8_t code, int32_t parameter)
{
  if (parameter != MISSING_PARAM) {
      serial_exchange_data.control.sm_modified = serial_exchange_data.control.stepper_p != parameter;
      serial_exchange_data.control.stepper_p = parameter;
  }
}


static
void rx_handler_debug_output(uint8_t code, int32_t parameter)
{
  if (parameter != MISSING_PARAM)
    serial_exchange_data.control.sm_debug_output = parameter & 3;
  else
    serial_exchange_data.control.sm_debug_output = serial_exchange_data.control.sm_debug_output ? 0 : 1;
}

struct {
  int32_t *value;
  char *desc;
} param_desc[] = {
    {
	.desc = "Static pressure",
	.value = &serial_exchange_data.control.static_pressure,
    },
    {
	.desc = "Pitot pressure",
	.value = &serial_exchange_data.control.pitot_pressure,
    },
    {
	.desc = "Tolerance/static",
	.value = &serial_exchange_data.control.tolerance_s,
    },
    {
	.desc = "Tolerance/pitot ",
	.value = &serial_exchange_data.control.tolerance_p,
    },
    {
	.desc = "Stepper max frequency",
	.value = &serial_exchange_data.control.stepper_freq,
    },
    {
	.desc = "Stepper speed coefficient",
	.value = &serial_exchange_data.control.stepper_p,
    },
};


static
void rx_handler_display_parameters(uint8_t code, int32_t parameter)
{
  unsigned i = 0;
  for (i = 0; i < lengthof(param_desc); i++) {
      int n = sprintf((char *)cdc_buf, "%s\t%ld\r\n", param_desc[i].desc, *(param_desc[i].value));
      CDC_Transmit_FS(cdc_buf, n);
      HAL_Delay(CDC_OUTPUT_DELAY);
  }
}

struct {
  int32_t *value;
  int32_t div;
  char *desc;
} sensor_data_desc[] = {
    {
	.desc = "Static pressure, Pa - BMP5/1",
	.div = 100,
	.value = (int32_t*)&bmp5_data[0].pressure,
    },
    {
	.desc = "Temperature, C - BMP5/1 ",
	.div = 100,
	.value = (int32_t*)&bmp5_data[0].temperature,
    },
    {
	.desc = "Static pressure, Pa - BMP5/2",
	.div = 100,
	.value = (int32_t*)&bmp5_data[1].pressure,
    },
    {
	.desc = "Temperature, C - BMP5/2 ",
	.div = 100,
	.value = (int32_t*)&bmp5_data[1].temperature,
    },
    {
	.desc = "Pitot pressure,  Pa - MS5525D",
	.div = 100,
	.value = (int32_t*)&ms5525_data.pressure,
    },
    {
	.desc = "Temperature, C - MS5525D",
	.div = 100,
	.value = (int32_t*)&ms5525_data.temperature,
    },
};

static
void rx_handler_display_sensors(uint8_t code, int32_t parameter)
{
  if (parameter == MISSING_PARAM)
    parameter = 1;

  unsigned i = 0;
  for (i = 0; i < lengthof(sensor_data_desc); i++) {
      int c1, c2;
      if (i % 2) {
	  c1 = '\r';
	  c2 = '\n';
      } else {
	  c1 = ' ';
	  c2 = '\t';
      }
      int32_t i_value = *(sensor_data_desc[i].value);
      int f_value = abs(i_value % sensor_data_desc[i].div);
      i_value /= sensor_data_desc[i].div;
      int n;
      if (0 != parameter) {
	  /* Human readable output */
	  n = sprintf((char *)cdc_buf, "%s\t%6ld.%02d\%c%c", sensor_data_desc[i].desc, i_value, f_value, c1, c2);
      } else {
	  /* Un-annotated output */
	  n = sprintf((char *)cdc_buf, "%6ld.%02d\r\n", i_value, f_value);
      }
      CDC_Transmit_FS(cdc_buf, n);
      HAL_Delay(CDC_OUTPUT_DELAY);
  }
}




static void rx_handler_help_text(uint8_t code, int32_t parameter);

static const char help_text[] = "Input format:[HH]C[+-0..9], where\n\r"
    "\tHH\t2 hex digits of CRC8 code (optional)\n\r"
    "\tC\tOne letter command code, see below\n\r"
    "\t+-0..9\tSigned decimal integer parameter (up to 9 digits)\r\n\r\n"
    "Supported commands [expected parameter]:\r\n";

struct rx_handlers_s {
  uint8_t code;
  char *desc;
  void (*handler)(uint8_t code, int32_t parameter);
} rx_handlers[] = {
    {
	'h',
	"\t\tDisplay this help text",
	rx_handler_help_text
    },
    {
	's',
	"[0..9]  \tSet static pressure, Pa",
	rx_handler_static_pressure
    },
    {
	'm',
	"[0..9]  \tStepper max frequency",
	rx_handler_stepper_freq,
    },
    {
	'n',
	"[0..9]  \tStepper speed coefficient",
	rx_handler_stepper_coef,
    },
    {
	'i',
	"[+-0..9]\tSet Pitot pressure, Pa",
	rx_handler_pitot_pressure
    },
    {
	'p',
	"\t\tDisplay current parameters",
	rx_handler_display_parameters
    },
    {
	'r',
	"[0..9]  \tTolerance/static, Pa",
	rx_handler_pressure_tolerance_s
    },
    {
	't',
	"[0..9]  \tTolerance/pitot, Pa",
	rx_handler_pressure_tolerance_p
    },
    {
	'o',
	"[1|0]\t\tOutput current sensors\' data (human-readable on|off)",
	rx_handler_display_sensors
    },
    {
	'v',
	"[1|0]\t\tPrint state transitions, on|off",
	rx_handler_debug_output
    }
};


unsigned rx_handlers_count = lengthof(rx_handlers);


static
void rx_handler_help_text(uint8_t code, int32_t parameter)
{
  int n = sprintf((char *)cdc_buf, "%s", help_text);
  CDC_Transmit_FS(cdc_buf, n);
  HAL_Delay(CDC_OUTPUT_DELAY);
  unsigned i = 0;
  for (i = 0; i < lengthof(rx_handlers); i++) {
//      n = sprintf((char *)cdc_buf, "\t%c%s\r\n", rx_handlers[i].code, rx_handlers[i].desc);
//      CDC_Transmit_FS(cdc_buf, n);
//      HAL_Delay(CDC_OUTPUT_DELAY);
      usb_cdc_printf("\t%c%s\r\n", rx_handlers[i].code, rx_handlers[i].desc);
  }
}


/*
 * Input stream parsing code. Reads text from the ring buffer, parses the commands and
 * checks validity.
 * Dispatches correctly formatted commands to the matching handlers from the table.
 * N.B. Command parser and ring buffer implementation that follow this comment
 * were produced using ChatGPT 5.4.
 */

/*
 * Ring buffer implementation
 */

#if (SERIAL_RX_BUFFER_SIZE & (SERIAL_RX_BUFFER_SIZE - 1)) != 0
# error "SERIAL_RX_BUFFER_SIZE must be a power of two"
#endif

#define RX_BUFFER_MASK   (SERIAL_RX_BUFFER_SIZE - 1)

static inline uint16_t rx_buffer_len(const struct rx_buffer_data_s *b)
{
    return (uint16_t)((b->head - b->tail) & RX_BUFFER_MASK);
}

static inline uint8_t rx_buffer_put(struct rx_buffer_data_s *b, uint8_t c)
{
    int16_t next = (b->head + 1) & RX_BUFFER_MASK;

    if (next == b->tail) {
        return 0;   /* full */
    }

    b->buffer[b->head] = c;
    b->head = next;
    return 1;
}

static inline uint8_t rx_buffer_get(struct rx_buffer_data_s *b, uint8_t *c)
{
    if (b->head == b->tail) {
        return 0;   /* empty */
    }

    *c = b->buffer[b->tail];
    b->tail = (b->tail + 1) & RX_BUFFER_MASK;
    return 1;
}


/* ---------------- CRC-8 ----------------
*  Polynomial x^8 + x^5 + x^4 + 1 = 0x31
*  init = 0x00, no reflection, no xorout
*/

static uint8_t rx_crc8_update(uint8_t crc, uint8_t data)
{
    uint8_t i;

    crc ^= data;
    for (i = 0; i < 8; i++) {
        crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0x31u) : (uint8_t)(crc << 1);
    }

    return crc;
}


/* ---------------- Parser ---------------- */

enum {
    RX_PARSE_WAIT = 0,
    RX_PARSE_CODE,
    RX_PARSE_PARAM,
    RX_PARSE_REJECT
};

static struct rx_parser_s rx_parser_state = {
    .state = RX_PARSE_WAIT
};


static inline uint8_t rx_is_space(uint8_t c)
{
    return (uint8_t)(c <= 32u);
}

static inline uint8_t rx_is_digit(uint8_t c)
{
    return (uint8_t)(c >= '0' && c <= '9');
}

static inline uint8_t rx_is_hex(uint8_t c)
{
    return (uint8_t)(
        (c >= '0' && c <= '9') ||
        (c >= 'A' && c <= 'F') ||
        (c >= 'a' && c <= 'f')
    );
}

static inline uint8_t rx_hex_value(uint8_t c)
{
    if (c <= '9') {
        return (uint8_t)(c - '0');
    }
    if (c <= 'F') {
        return (uint8_t)(c - 'A' + 10u);
    }
    return (uint8_t)(c - 'a' + 10u);
}

static inline void rx_parser_reset_wait(struct rx_parser_s *p)
{
    p->state = RX_PARSE_WAIT;
    p->crc_present = 0;
    p->crc_expected = 0;
    p->crc = 0;
    p->code = 0;
    p->sign = 0;
    p->digits = 0;
    p->got_sign = 0;
    p->crc_digit_count = 0;
    p->value = 0;
}

static inline void rx_parser_reset_reject(struct rx_parser_s *p)
{
    p->state = RX_PARSE_REJECT;
    p->crc_present = 0;
    p->crc_expected = 0;
    p->crc = 0;
    p->code = 0;
    p->sign = 0;
    p->digits = 0;
    p->got_sign = 0;
    p->crc_digit_count = 0;
    p->value = 0;
}

static inline void rx_parser_start_code(struct rx_parser_s *p, uint8_t c)
{
    p->crc = rx_crc8_update(0, c);
    p->code = c;
    p->sign = 0;
    p->digits = 0;
    p->got_sign = 0;
    p->value = 0;
    p->state = RX_PARSE_PARAM;
}

static inline void rx_parser_start_token(struct rx_parser_s *p, uint8_t c)
{
    if (rx_is_hex(c)) {
        p->crc_present = 1;
        p->crc_expected = rx_hex_value(c);
        p->crc_digit_count = 1;
        p->state = RX_PARSE_CODE;
    } else {
        p->crc_present = 0;
        p->crc_digit_count = 0;
        rx_parser_start_code(p, c);
    }
}

static void rx_dispatch(uint8_t code, int32_t parameter)
{
    unsigned i;

    for (i = 0; i < rx_handlers_count; i++) {
        if (rx_handlers[i].code == code) {
            rx_handlers[i].handler(code, parameter);
            break;
        }
    }
}

static inline void rx_parser_finish(struct rx_parser_s *p)
{
    int32_t param;

    if (p->crc_present && (p->crc != p->crc_expected)) {
        rx_parser_reset_reject(p);
        return;
    }

    if (p->digits == 0) {
	  param = MISSING_PARAM;
    } else {
	param = p->sign ? -(int32_t)p->value : (int32_t)p->value;
    }
    rx_dispatch(p->code, param);
    rx_parser_reset_wait(p);
}

/* Call repeatedly from main loop */
static void rx_parse_buffer(struct rx_buffer_data_s *rb, struct rx_parser_s *p)
{
    uint8_t c;

    while (rx_buffer_get(rb, &c)) {

        switch (p->state) {

        case RX_PARSE_WAIT:
            if (!rx_is_space(c)) {
                rx_parser_start_token(p, c);
            }
            break;

        case RX_PARSE_CODE:
            if (p->crc_present) {
                if (p->crc_digit_count == 1) {
                    if (!rx_is_hex(c)) {
                        rx_parser_reset_reject(p);
                        break;
                    }
                    p->crc_expected = (uint8_t)((p->crc_expected << 4) | rx_hex_value(c));
                    p->crc_digit_count = 2;
                    break;
                }

                if (rx_is_space(c) || rx_is_hex(c)) {
                    rx_parser_reset_reject(p);
                    break;
                }

                rx_parser_start_code(p, c);
                break;
            }

            rx_parser_reset_reject(p);
            break;

        case RX_PARSE_PARAM:
            if (rx_is_space(c)) {
                rx_parser_finish(p);
                break;
            }

            if (!p->got_sign && !p->digits && (c == '+' || c == '-')) {
                p->got_sign = 1;
                p->sign = (uint8_t)(c == '-');
                p->crc = rx_crc8_update(p->crc, c);
                break;
            }

            if (rx_is_digit(c)) {
                if (p->digits >= 9) {
                    rx_parser_reset_reject(p);
                    break;
                }

                p->value = p->value * 10u + (uint32_t)(c - '0');
                p->digits++;
                p->crc = rx_crc8_update(p->crc, c);
                break;
            }

            rx_parser_reset_reject(p);
            break;

        case RX_PARSE_REJECT:
            if (rx_is_space(c)) {
                rx_parser_reset_wait(p);
            }
            break;

        default:
            rx_parser_reset_reject(p);
            break;
        }
    }
}

