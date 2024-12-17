#ifndef INCLUDE_MOTORS_H
#define INCLUDE_MOTORS_H

#include <stdint.h>


void setup_motors(void);
void motor_steering_drive(float angular_vel);
void motor_rear_left_drive(float angular_vel);
void motor_rear_right_drive(float angular_vel);

#endif  // #ifndef INCLUDE_MOTORS_H

