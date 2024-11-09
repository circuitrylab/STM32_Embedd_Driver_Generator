/*
 * led.h
 *
 *  Created on: Oct 25, 2024
 *      Author: jdvil
 */

#ifndef LED_H_
#define LED_H_

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

void ledInit(TIM_HandleTypeDef *_tim);
void ledUpdate(uint16_t ledPeriodHigh, uint16_t ledPeriodLow);

#ifdef __cplusplus
}
#endif



#endif /* LED_H_ */
