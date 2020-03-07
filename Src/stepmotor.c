/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : stepmotor.h
  * @brief          : Nguyen Trung Hieu
  *                   hieunt91@viettel.com.vn
  *                   Dai Hoc Dien Luc
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stepmotor.h"
#include "stdbool.h"
#include "stdio.h"

uint16_t count1 = 0;
int flagStep = 0;
uint8_t oldStatusFlagStep = 0;
void directionOfRotation(bool direction, int degree) {
	uint16_t t = degree*512/360;
	for (uint16_t i = 0; i <t; i++) {
		if(direction) {
			HAL_GPIO_WritePin (IN1_Port, IN1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin (IN2_Port, IN2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin (IN3_Port, IN3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin (IN4_Port, IN4_Pin, GPIO_PIN_RESET);

			HAL_Delay(delayTime);

			HAL_GPIO_WritePin (IN1_Port, IN1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin (IN2_Port, IN2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin (IN3_Port, IN3_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin (IN4_Port, IN4_Pin, GPIO_PIN_RESET);

			HAL_Delay(delayTime);

			HAL_GPIO_WritePin (IN1_Port, IN1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin (IN2_Port, IN2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin (IN3_Port, IN3_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin (IN4_Port, IN4_Pin, GPIO_PIN_SET);

			HAL_Delay(delayTime);

			HAL_GPIO_WritePin (IN1_Port, IN1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin (IN2_Port, IN2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin (IN3_Port, IN3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin (IN4_Port, IN4_Pin, GPIO_PIN_SET);

			HAL_Delay(delayTime);
		} else {
			HAL_GPIO_WritePin (IN1_Port, IN1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin (IN2_Port, IN2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin (IN3_Port, IN3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin (IN4_Port, IN4_Pin, GPIO_PIN_SET);

			HAL_Delay(delayTime);

			HAL_GPIO_WritePin (IN1_Port, IN1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin (IN2_Port, IN2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin (IN3_Port, IN3_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin (IN4_Port, IN4_Pin, GPIO_PIN_SET);

			HAL_Delay(delayTime);

			HAL_GPIO_WritePin (IN1_Port, IN1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin (IN2_Port, IN2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin (IN3_Port, IN3_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin (IN4_Port, IN4_Pin, GPIO_PIN_RESET);

			HAL_Delay(delayTime);

			HAL_GPIO_WritePin (IN1_Port, IN1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin (IN2_Port, IN2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin (IN3_Port, IN3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin (IN4_Port, IN4_Pin, GPIO_PIN_RESET);

			HAL_Delay(delayTime);
		}
	}
}
void turnOn() {
	flagStep = 1;
}
void miniStep(bool direction) {
	if (flagStep == 1) {
		for (uint16_t i = 0; i < 10; i++) {
			if(direction) {
				HAL_GPIO_WritePin (IN1_Port, IN1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin (IN2_Port, IN2_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin (IN3_Port, IN3_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin (IN4_Port, IN4_Pin, GPIO_PIN_RESET);

				HAL_Delay(delayTime);

				HAL_GPIO_WritePin (IN1_Port, IN1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin (IN2_Port, IN2_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin (IN3_Port, IN3_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin (IN4_Port, IN4_Pin, GPIO_PIN_RESET);

				HAL_Delay(delayTime);

				HAL_GPIO_WritePin (IN1_Port, IN1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin (IN2_Port, IN2_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin (IN3_Port, IN3_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin (IN4_Port, IN4_Pin, GPIO_PIN_SET);

				HAL_Delay(delayTime);

				HAL_GPIO_WritePin (IN1_Port, IN1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin (IN2_Port, IN2_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin (IN3_Port, IN3_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin (IN4_Port, IN4_Pin, GPIO_PIN_SET);

				HAL_Delay(delayTime);
			}
			else {
				HAL_GPIO_WritePin (IN1_Port, IN1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin (IN2_Port, IN2_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin (IN3_Port, IN3_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin (IN4_Port, IN4_Pin, GPIO_PIN_SET);

				HAL_Delay(delayTime);

				HAL_GPIO_WritePin (IN1_Port, IN1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin (IN2_Port, IN2_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin (IN3_Port, IN3_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin (IN4_Port, IN4_Pin, GPIO_PIN_SET);

				HAL_Delay(delayTime);

				HAL_GPIO_WritePin (IN1_Port, IN1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin (IN2_Port, IN2_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin (IN3_Port, IN3_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin (IN4_Port, IN4_Pin, GPIO_PIN_RESET);

				HAL_Delay(delayTime);

				HAL_GPIO_WritePin (IN1_Port, IN1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin (IN2_Port, IN2_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin (IN3_Port, IN3_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin (IN4_Port, IN4_Pin, GPIO_PIN_RESET);

				HAL_Delay(delayTime);
			}
		}
		count1 ++;
		if (count1 == 40) {
			count1 = 0;
			flagStep = 0;
			HAL_GPIO_WritePin (IN1_Port, IN1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin (IN2_Port, IN2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin (IN3_Port, IN3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin (IN4_Port, IN4_Pin, GPIO_PIN_RESET);
		}
	}
}
