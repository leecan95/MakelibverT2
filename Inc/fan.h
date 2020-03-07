/* Author: TTTBYT
 *
 * Brief: ERV Motor Control.
 * Two fan: FAN1 & FAN2 (2 signal wire: PWM-speed cotrol and TACH-speed feedback (2 or 3 ppr) )
 * Pin connect:
 * 		FAN1_TACH: PA6 (TIM3_CH1_IC)
 * 		FAN2_TACH: PA7 (TIM3_CH2_IC)
 
 * 		FAN1_PWM:  PA8 (TIM1_CH1_PWM)
 * 		FAN2_PWM:  PA11(TIM1_CH4_PWM)
 *
 * 	Program:
 * 	 1. Add fan.h and fan.c
 * 	 2. In CubeMX, config: TIM1_CH1_PWM and TIM1_CH4_PWM
 * 	 3. Add fan_init();
 * 	 4. Using stop, run... -> speed: 0 -> 100
 */

#ifndef FAN_H
#define FAN_H

#include "stm32f4xx.h"

extern TIM_HandleTypeDef htim1;

//Motor init, fix in use TIM3 (for speed control) and TIM1 (for speed feedback)
void fan_init();

/* Motor 1 */
void fan_1_stop();
void fan_1_run(uint8_t level);	// level: 0-100

/* Motor 2 */
void fan_2_stop();
void fan_2_run(uint8_t level);	// level: 0-100

void fan_12_run(uint8_t level);
#endif
