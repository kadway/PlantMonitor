/*
 * helpers.c
 *
 *  Created on: May 8, 2019
 *      Author: johny
 */
#include "helpers.h"

void Delay(__IO uint32_t nCount)
{
  while(nCount--)
  {
  }
}

uint16_t ADC_Read(void)
{
	// Start ADC conversion
	//ADC_SoftwareStartConv(ADC3);
	// Wait until conversion is finish
	while (!ADC_GetFlagStatus(ADC3, ADC_FLAG_EOC));
    return ADC_GetConversionValue(ADC3);
}
