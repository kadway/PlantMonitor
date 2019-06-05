/*
 * platform.h
 *
 *  Created on: May 8, 2019
 *      Author: johny
 */

#ifndef INC_PLATFORM_H_
#define INC_PLATFORM_H_
#include <stdio.h>
#include <stdlib.h>
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#include "stm32_ub_uart.h"
#include "rtc.h"
#include "helpers.h"

//#define SETTIME //define to set the hardcoded time on startup


void initHW();
void ADC_Config(uint16_t *ADC3ReservedMemory);
uint16_t* getSensorValues(void);
void I2C_Config(void);
void Config_Wakeup_INT(void);
void PrepareSleepMode(void);
void PrepareRunMode(void);

void StartSleep(void);

void SysTick_Configuration(void);;
void RTC_Config(void);




/*
 * ADC Pins Available
 *
 * PC0 - ADC123_IN10
 * PC1 - ADC123_IN11
 * PC2 - ADC123_IN12
 * PC3 - ADC123_IN13
 * PC4 - ADC12_IN14
 * PC5 - ADC12_IN15
 * PA0 - ADC123_IN0 (also Discovery blue Push button)
 * PA1 - ADC123_IN1
 * PA2 - ADC123_IN2
 * PA3 - ADC123_IN3
 * PA4 - ADC12_IN4
 * PA5 - ADC12_IN5
 * PA6 - ADC12_IN6
 * PA7 - ADC12_IN7
 * PB0 - ADC12_IN8
 * PB1 - ADC12_IN9
 *
 */

//external wakeup interrupt
// PE7 - falling edge

/*
 * USART used available
 * PD8 - USART3_TX
 * PD9 - USART3_RX
 * PD10 - USART3 CK
 */

/*
 * I2C used
 * PB6 - I2C1 SCL
 * PB7 - I2C1 SDA
 *
 */

/*
 * SPI used
 *
 * PB4 - SPI1 MISO
 * PB5 - SPI1 MOSI
 * PB3 - SPI1 SCK
 *
 */

/*
 * GPIOs as outputs
 *
 * 16 for moisture sensor supply (one for each ADC channel)
 * PE 0,1,2,3,4,5,6
 * PD 0,1,2,3,4,5,6,7
 * PC 12
 *
 * 9  for solenoid water valve activation relay
 * PC 9,10,11
 * PA 8,9,10,13,14,15,
 *
 * 2 for water pump activation relay
 * PB 8,9
 *
 */

#endif /* INC_PLATFORM_H_ */
