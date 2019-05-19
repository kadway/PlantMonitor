/*
 * helpers.h
 *
 *  Created on: May 8, 2019
 *      Author: johny
 */

#ifndef INC_HELPERS_H_
#define INC_HELPERS_H_

#include "stm32f4xx_conf.h"

void Delay(__IO uint32_t nCount);

uint16_t ADC_Read(void);

#endif /* INC_HELPERS_H_ */
