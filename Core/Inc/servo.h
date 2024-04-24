# ifndef SERVO_H
# define SERVO_H

#include "stm32f0xx_hal.h"

void servoUtil_configServo();
void servoUtil_setServo(uint8_t angle);

#endif