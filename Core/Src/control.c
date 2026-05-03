/*
 * control.c
 *
 *  Created on: Apr 5, 2026
 *      Author: shir
 *
 * The control algorithm in this file is based on a finite dynamical system idea.
 * Its components are binary states derived from sensors relative to thresholds and
 * output pin levels. Hidden states that track internal conditions are also allowed.
 * These binary states form a finite Boolean state space.
 * The update function is represented by the ordered chain of rules, applied sequentially.
 * The rule engine computes the fixed point of this ordered update function on the
 * Boolean state space represented a bit vector.
 * In this implementation there is no cycle prevention at runtime. Cyclic attractor
 * and non-terminating loops constitute a flaw in the update function and must be detected
 * and fixed before the production run.
 */

#include "stdlib.h"
#include "main.h"
#include "control.h"
#include "serial.h"

enum {
  SM_SENSORS_ACTIVE = 0,        // All sensors are active
  SM_STATIC_LOW,                // Static pressure is too low and outside tolerance range
  SM_STATIC_LOW_TOL,            // Static pressure is below set point but within tolerance
  SM_STATIC_HIGH_TOL,           // Static pressure is above set point but within tolerance
  SM_STATIC_HIGH,               // Static pressure is too high and outside tolerance range

  SM_PITOT_LOW,                // Pitot pressure is too low and outside tolerance range
  SM_PITOT_LOW_TOL,            // Pitot pressure is below set point but within tolerance
  SM_PITOT_HIGH_TOL,           // Pitot pressure is above set point but within tolerance
  SM_PITOT_HIGH,               // Pitot pressure is too high and outside tolerance range

  SM_NEAR_ATM_PRESS,           // Static pressure is close to atmospheric, will need pressurizing pump

  SM_EXVALVE_ON,               // Exhaust valve is on
  SM_DIFVALVE_ON,              // Differential control leak valve is on
  SM_VACPUMP_ON,               // Vacuum pump is on
  SM_STEPPER_ON,               // Differential valve actuator is on
  SM_STEPPER_POS_DIR,          // Differential valve actuator positive direction is selected
  SM_PRESSPUMP_ON,             // Pressurizing pump is on

  SM_STATE_COUNT,
};

const char sm_state_idstr[SM_STATE_COUNT][8] = {
    "SACTIVE",

    "STA_LOW", "ST_LTOL", "ST_HTOL", "ST_HIGH",
    "PIT_LOW", "PT_LTOL", "PT_HTOL", "PT_HIGH",

    "NEARATM",
    "XVAL_ON", "DVAL_ON", "VACP_ON", "STEP_ON", "STP_POS", "PRSP_ON"
};


/*
 * Helper macro. Convert bit index to bit mask.
 */
#define _M(x) (1u << (x))

static const uint32_t VALVE_ACTIVE_PIN_STATE = 0;
static const uint32_t VACPUMP_ACTIVE_PIN_STATE = 0;
static const uint32_t PRESSPUMP_ACTIVE_PIN_STATE = 0;
static const uint32_t STEPPER_POSITIVE_DIRECTION_PIN_STATE = 0;

/*
 * State machine definition.
 * State machine is defined as an array of conditional rules that are applied sequentially
 * until no more matching rules can be found.
 * Each rule consists of a condition and an action.
 * Conditions and actions are defined as pairs of bit masks, inverted and non-inverted.
 * For conditions, inverted mask defines bits that should currently be clear in the state word,
 * and non-inverted mask defines bits that should be set in the state word.
 * For actions, inverted mask defines bits that should be cleared (assigned zero) if the rule applies,
 * and non-inverted mask defines bits that should be set to 1.
 *
 * Zero bits in all masks mean that corresponding bits in the state word should be ignored.
 */

struct state_machine_rules_s {
  uint32_t neg_condition;
  uint32_t pos_condition;

  uint32_t neg_action;
  uint32_t pos_action;
} sm_rules[] = {
    {
	/* If one or more sensors are not active all motors should be stopped. Valve state is unchanged. */
	.neg_condition = _M(SM_SENSORS_ACTIVE),
	0,
	.neg_action = _M(SM_VACPUMP_ON) + _M(SM_STEPPER_ON) + _M(SM_PRESSPUMP_ON),
	0
    },

    {
	/* Static pressure is too low. Stop the pump, release the valve. */
	0,
	.pos_condition = _M(SM_SENSORS_ACTIVE) + _M(SM_STATIC_LOW),
	.neg_action = _M(SM_VACPUMP_ON),
	.pos_action = _M(SM_EXVALVE_ON) + _M(SM_DIFVALVE_ON),
    },

    {
	/* Static pressure is low, but within tolerance. Stop the pump. */
	0,
	.pos_condition = _M(SM_SENSORS_ACTIVE) + _M(SM_STATIC_LOW_TOL),
	.neg_action = _M(SM_VACPUMP_ON),
	.pos_action = 0,
    },

    {
	/* Static pressure is high, but within tolerance. Shut the valve. */
	0,
	.pos_condition = _M(SM_SENSORS_ACTIVE) + _M(SM_STATIC_HIGH_TOL),
	.neg_action = _M(SM_EXVALVE_ON),
	.pos_action = 0,
    },

    {
	/* Static pressure is too high. Run the pump, shut the valve. */
	0,
	.pos_condition = _M(SM_SENSORS_ACTIVE) + _M(SM_STATIC_HIGH),
	.neg_action = _M(SM_EXVALVE_ON),
	.pos_action = _M(SM_VACPUMP_ON) + _M(SM_DIFVALVE_ON),
    },

    {
	/* Pitot pressure is too low. Run stepper to positive direction. */
	0,
	.pos_condition = _M(SM_SENSORS_ACTIVE) + _M(SM_PITOT_LOW),
	.neg_action = 0,
	.pos_action = _M(SM_STEPPER_POS_DIR) + _M(SM_STEPPER_ON) + _M(SM_DIFVALVE_ON),
    },

    {
	/* Pitot pressure is low, but within tolerance. Stop stepper if it was moving in negative direction. */
	.neg_condition = _M(SM_STEPPER_POS_DIR),
	.pos_condition = _M(SM_SENSORS_ACTIVE) + _M(SM_PITOT_LOW_TOL),
	.neg_action = _M(SM_STEPPER_ON),
	.pos_action = 0,
    },

    {
	/* Pitot pressure is high, but within tolerance. Stop stepper if it was moving in positive direction. */
	0,
	.pos_condition = _M(SM_SENSORS_ACTIVE) + _M(SM_PITOT_HIGH_TOL) + _M(SM_STEPPER_POS_DIR),
	.neg_action = _M(SM_STEPPER_ON),
	.pos_action = 0,
    },

    {
	/* Pitot pressure is too high. Run stepper in negative direction. */
	0,
	.pos_condition = _M(SM_SENSORS_ACTIVE) + _M(SM_PITOT_HIGH),
	.neg_action = _M(SM_STEPPER_POS_DIR),
	.pos_action = _M(SM_STEPPER_ON) + _M(SM_DIFVALVE_ON),
    },

    {
	/* Static pressure is very close to atmospheric, start the pressurizing pump */
	0,
	.pos_condition = _M(SM_SENSORS_ACTIVE) + _M(SM_NEAR_ATM_PRESS) + _M(SM_DIFVALVE_ON),
	.neg_action = 0,
	.pos_action = _M(SM_PRESSPUMP_ON),
    },

    {
	/* Static pressure is below atmospheric, stop the pressurizing pump */
	.neg_condition = _M(SM_NEAR_ATM_PRESS),
	.pos_condition = 0,
	.neg_action = _M(SM_PRESSPUMP_ON),
	.pos_action = 0,
    },

    {
	/* Differential valve is closed, stop the pressurizing pump */
	.neg_condition = _M(SM_DIFVALVE_ON),
	.pos_condition = 0,
	.neg_action = _M(SM_PRESSPUMP_ON),
	.pos_action = 0,
    },

    {
        /* Close differential valve leak inlets when no other pressure changing device is active */
	.pos_condition = 0,
	.neg_condition = _M(SM_STEPPER_ON) + _M(SM_VACPUMP_ON) + _M(SM_EXVALVE_ON),
	.neg_action = _M(SM_DIFVALVE_ON),
	.pos_action = 0,
    },
};


/*
 * Current state of the state machine
 */
static uint32_t sm_vector_current = 0;

static uint8_t current_debug_output = 0;

/*
 * Helper functions
 */
inline static void set_sm_state(int smbit, bool onoff)
{
  if (onoff) {
      sm_vector_current |= _M(smbit);
  } else {
      sm_vector_current &= ~_M(smbit);
  }
}

inline static bool get_sm_state(int smbit)
{
  return 0 != (sm_vector_current & _M(smbit));
}

inline static void set_stepper(bool onoff, bool positive)
{
  if (onoff) {

/* Set direction pin before starting pulses */
      if (positive ^ (0 == STEPPER_POSITIVE_DIRECTION_PIN_STATE)) {
	  LL_GPIO_SetOutputPin(LED_PIN_GPIO_Port, LED_PIN_Pin);
      } else {
	  LL_GPIO_ResetOutputPin(LED_PIN_GPIO_Port, LED_PIN_Pin);
      }

      /* Calculate proportional motor speed */
      int32_t pitot_delta = abs(ms5525_data.pressure - serial_exchange_data.control.pitot_pressure);

      uint32_t max_freq = serial_exchange_data.control.stepper_freq;

      uint32_t freq = max_freq * pitot_delta * serial_exchange_data.control.stepper_p / 256;

      /* Apply limits */
      if (freq < max_freq / 20)
	freq = max_freq / 20;

      if (freq > max_freq)
	freq = max_freq;

/* The timer that drives stepper output uses microsecond timer as clock source. */
      int32_t stepper_reload_value = 1000000 / freq;
/* Set compare value at 1/2 of the reload value to generate 50% duty cycle pulses */
      int32_t stepper_compare_value = stepper_reload_value / 2;

      LL_TIM_SetAutoReload(StepperTimer, stepper_reload_value);
      LL_TIM_OC_SetCompareCH_Stepper(StepperTimer, stepper_compare_value);
      LL_TIM_CC_EnableChannel(StepperTimer, StepperChannel);
  } else {
/* Stop pulses */
      LL_TIM_SetAutoReload(StepperTimer, 0);
      LL_TIM_CC_DisableChannel(StepperTimer, StepperChannel);
  }
}

control_output_t control_output = {
    0,
};

/*
 * Read external variables and convert to machine states.
 */
static
void read_input_signals(void)
{
  int32_t static_pressure = bmp5_data[STATIC_PRESSURE_SENSOR_INDEX].pressure / 100;
  int32_t atm_pressure = bmp5_data[AMBIENT_PRESSURE_SENSOR_INDEX].pressure / 100;
  int32_t pitot_pressure = ms5525_data.pressure;

  int32_t static_delta = static_pressure - serial_exchange_data.control.static_pressure;
  int32_t pitot_delta = pitot_pressure - serial_exchange_data.control.pitot_pressure;

  int32_t static_atm_delta = atm_pressure - static_pressure;

  set_sm_state(SM_SENSORS_ACTIVE,  (HAL_OK == sensors_active));

  set_sm_state(SM_STATIC_LOW,      (static_delta < -serial_exchange_data.control.tolerance));
  set_sm_state(SM_STATIC_LOW_TOL,  (static_delta < 0) && (static_delta > -serial_exchange_data.control.tolerance));
  set_sm_state(SM_STATIC_HIGH_TOL, (static_delta > 0) && (static_delta <  serial_exchange_data.control.tolerance));
  set_sm_state(SM_STATIC_HIGH,     (static_delta > serial_exchange_data.control.tolerance));

  set_sm_state(SM_PITOT_LOW,      (pitot_delta < -serial_exchange_data.control.tolerance));
  set_sm_state(SM_PITOT_LOW_TOL,  (pitot_delta < 0) && (pitot_delta > -serial_exchange_data.control.tolerance));
  set_sm_state(SM_PITOT_HIGH_TOL, (pitot_delta > 0) && (pitot_delta <  serial_exchange_data.control.tolerance));
  set_sm_state(SM_PITOT_HIGH,     (pitot_delta > serial_exchange_data.control.tolerance));

  set_sm_state(SM_NEAR_ATM_PRESS, static_atm_delta < atm_pressure/16);

}

/*
 * Output control signals via GPIO/timers according
 * to the computed machine states.
 */
static
void set_output_signals(void)
{
  if (get_sm_state(SM_EXVALVE_ON) ^ (0 == VALVE_ACTIVE_PIN_STATE)) {
      LL_GPIO_SetOutputPin(VALVE1_GPIO_Port, VALVE1_Pin);
  } else {
      LL_GPIO_ResetOutputPin(VALVE1_GPIO_Port, VALVE1_Pin);
  }

  if (get_sm_state(SM_DIFVALVE_ON) ^ (0 == VALVE_ACTIVE_PIN_STATE)) {
      LL_GPIO_SetOutputPin(VALVE2_GPIO_Port, VALVE2_Pin);
  } else {
      LL_GPIO_ResetOutputPin(VALVE2_GPIO_Port, VALVE2_Pin);
  }

  if (get_sm_state(SM_VACPUMP_ON) ^ (0 == VACPUMP_ACTIVE_PIN_STATE)) {
      LL_GPIO_SetOutputPin(PUMP1_GPIO_Port, PUMP1_Pin);
  } else {
      LL_GPIO_ResetOutputPin(PUMP1_GPIO_Port, PUMP1_Pin);
  }

  if (get_sm_state(SM_PRESSPUMP_ON) ^ (0 == PRESSPUMP_ACTIVE_PIN_STATE)) {
      LL_GPIO_SetOutputPin(PUMP1_GPIO_Port, PUMP2_Pin);
  } else {
      LL_GPIO_ResetOutputPin(PUMP1_GPIO_Port, PUMP2_Pin);
  }

  set_stepper(get_sm_state(SM_STEPPER_ON),
		get_sm_state(SM_STEPPER_POS_DIR));
}

/*
 * Helper function to debug state transitions.
 * Print states that were switched off (indicated with -> prefix) or on (<- prefix).
 */
static uint32_t debug_old_state = 0;

static
void debug_state_transitions(uint32_t new_state)
{

  uint32_t off_states = (debug_old_state ^ new_state) & debug_old_state;
  uint32_t on_states = (debug_old_state ^ new_state) & new_state;
  debug_old_state = new_state;

  unsigned i;
  for (i = 0; i < SM_STATE_COUNT; i++, off_states >>= 1, on_states >>= 1) {
      const char *idstr = sm_state_idstr[i];
      const char *fmt = NULL;
      if (off_states & 1) {
	  fmt = "\to<%s\r\n";
      }
      if (on_states & 1) {
	  fmt = ">|%s\r\n";
      }
      if (fmt && idstr)
	  usb_cdc_printf(fmt, idstr);
  }

}

void run_control_loop(void)
{
//  uint32_t sm_save_state = sm_vector_current;
  uint32_t sm_previous;

  /* Output current states if debug output was requested */
  if ((current_debug_output == 0) != (0 == serial_exchange_data.control.sm_debug_output)) {
      if (0 == current_debug_output) {
	  debug_old_state = 0;
	  debug_state_transitions(sm_vector_current);
      }
      current_debug_output = serial_exchange_data.control.sm_debug_output != 0;
  }

  read_input_signals();
  do {
      /* Display state transitions if debug output is requested. */
      if (serial_exchange_data.control.sm_debug_output)
	  debug_state_transitions(sm_vector_current);

      sm_previous = sm_vector_current;
      size_t i;
      for (i = 0; i < lengthof(sm_rules); i++) {
	  if (((sm_vector_current & sm_rules[i].pos_condition) == sm_rules[i].pos_condition) &&
	      ((sm_vector_current & sm_rules[i].neg_condition) == 0)) {
		  sm_vector_current = (sm_vector_current & (~sm_rules[i].neg_action)) | sm_rules[i].pos_action;
	      }
      }


  } while (sm_vector_current != sm_previous);
  set_output_signals();

}

