#ifndef INCLUDE_MOTORS_H
#define INCLUDE_MOTORS_H

#include <stdint.h>


void setup_hrtim(void);
void setup_motors(void);
void motor_steering_write(uint16_t period_ticks);
void motor_rear_left_write(uint16_t period_ticks);
void motor_rear_right_write(uint16_t period_ticks);

#endif  // #ifndef INCLUDE_MOTORS_H

