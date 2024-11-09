/*
 * led.c
 *
 *  Created on: Oct 25, 2024
 *      Author: jdvil
 */



#include "led.h"

TIM_HandleTypeDef *ledTimer = NULL;

void ledInit(TIM_HandleTypeDef *_tim)
{
	ledTimer = _tim;

	HAL_TIM_Base_Start_IT(ledTimer);
}

void ledUpdate(uint16_t ledPeriodHigh, uint16_t ledPeriodLow)
{
	if(HAL_GPIO_ReadPin(LED_GPIO_Port, LED_Pin) == GPIO_PIN_SET) {
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

		__HAL_TIM_SET_COUNTER(ledTimer, 0);
		__HAL_TIM_SET_AUTORELOAD(ledTimer, ledPeriodLow);

		HAL_TIM_Base_Start_IT(ledTimer);
	} else {
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

		__HAL_TIM_SET_COUNTER(ledTimer, 0);
		__HAL_TIM_SET_AUTORELOAD(ledTimer, ledPeriodHigh);

		HAL_TIM_Base_Start_IT(ledTimer);
	}
}
