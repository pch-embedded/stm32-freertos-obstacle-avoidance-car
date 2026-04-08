/* motor.h */
#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include "stm32h7xx_hal.h"

void Motor_Init(TIM_HandleTypeDef *htim);
void Motor_Run(uint8_t speed_step);
void Motor_Stop(void);
void Motor_Back(uint8_t speed_step);
void Motor_TurnLeft(uint8_t speed_step);
void Motor_TurnRight(uint8_t speed_step);
#endif /* MOTOR_H */
