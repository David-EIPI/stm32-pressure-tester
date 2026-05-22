/*
 * control.h
 *
 *  Created on: Apr 5, 2026
 *      Author: shir
 */

#ifndef SRC_CONTROL_H_
#define SRC_CONTROL_H_


/*
 * Output levels of the device control pins.
 */
typedef struct control_output_s {
  int valve1:1;
  int valve2:1;
  int pump1:1;
  int pump2:1;
} control_output_t;

typedef void (*sm_transition_cb)(uint32_t old_state, uint32_t new_state);

extern control_output_t control_output;

extern uint32_t dbg_stepper_freq;

void run_control_loop();

#endif /* SRC_CONTROL_H_ */
