/*
 * platform.c
 *
 *  Created on: May 8, 2019
 *      Author: johny
 */

#include "platform.h"

#ifdef SETTIME
struct tm time;
#endif


uint16_t ADC1Data[14];
RTC_InitTypeDef RTC_InitStructure;
RTC_TimeTypeDef RTC_TimeStructure;
RTC_AlarmTypeDef  RTC_AlarmStructure;


/* Private variables ---------------------------------------------------------*/
uint16_t ADCTripleConvertedValue[3]={0,0,0};

void initHW()
{

	SystemInit(); // Clock init
	// Init UART
	// Com2 115200 Baud
	UB_Uart_Init();
	//UB_Uart_SendString(COM3, "UART INIT", LFCR);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure2;
	// Init LED
	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);


	ADC1_Init(ADC1Data);// ADC config

	I2C_Config();//Configure and start I2c

	//init the ds3231 rtc
	rtc_init();
	//Config_Wakeup interrupt
	RTC_Config();
	//wakeup from user button pin PA0
	Config_Wakeup_INT();

}


uint16_t* getSensorValues(void){
	//return pointer for memory where ADC conversion values are stored by DMA
	return ADC1Data;
}

void I2C_Config(void){

	/*
	 * I2C used
	 * PB6 - I2C1 SCL
	 * PB7 - I2C1 SDA
	 *
	 */

	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef I2C_InitStruct;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1); //SCL
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1); //SDA


	I2C_DeInit(I2C1);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

	I2C_InitStruct.I2C_ClockSpeed = 400000; // 100kHz
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;   // I2C mode
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;   // 50% duty cycle --> standard
	I2C_InitStruct.I2C_OwnAddress1 = 0x0;   // own address, not relevant in master mode
	I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;   // Enable acknowledge when reading
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // set address length
	I2C_Init(I2C1, &I2C_InitStruct);   // init I2C1

    I2C_Cmd(I2C1, ENABLE);

}

void ADC1_Init(uint16_t *ADC1Data){
	ADC_InitTypeDef       ADC_InitStruct;
	ADC_CommonInitTypeDef ADC_CommonInitStruct;
	DMA_InitTypeDef       DMA_InitStruct;
	GPIO_InitTypeDef      GPIO_InitStruct;
	NVIC_InitTypeDef      NVIC_InitStructure;

	ADC_DeInit();
	/* Enable ADC1, DMA2 and GPIO clocks ****************************************/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	/* DMA2 Stream0 channel1 configuration **************************************/
	DMA_InitStruct.DMA_Channel = DMA_Channel_0;
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;//ADC1's data register
	DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)ADC1Data;
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStruct.DMA_BufferSize = 14;
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//Reads 16 bit values
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//Stores 16 bit values
	DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStruct.DMA_Priority = DMA_Priority_High;
	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream0, &DMA_InitStruct);
	DMA_Cmd(DMA2_Stream0, ENABLE);
	// wait for the DMA Stream to be ready
	 while(DMA_GetCmdStatus(DMA2_Stream0)==DISABLE){};

	DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, ENABLE);
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Configure GPIO pins ******************************************************/

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;//The pins are configured in analog mode
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL ;//We don't need any pull up or pull down

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5; //PC1, PC2, PC3, PC4, PC5
	GPIO_Init(GPIOC, &GPIO_InitStruct); //Initialize GPIOC pins with the configuration

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;//PA1, PA2, PA3, PA4, PA5, PA6, PA7
	GPIO_Init(GPIOA, &GPIO_InitStruct);//Initialize GPIOA pins with the configuration

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1; //PB0, PB1
	GPIO_Init(GPIOB, &GPIO_InitStruct);//Initialize GPIOB pins with the configuration

	/* ADC Common Init **********************************************************/
	ADC_CommonInitStruct.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStruct.ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_CommonInitStruct.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStruct.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStruct);

	/* ADC2 Init ****************************************************************/
	ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;//Input voltage is converted into a 12bit int (max 4095)
	ADC_InitStruct.ADC_ScanConvMode = ENABLE;//The scan is configured in multiple channels
	ADC_InitStruct.ADC_ContinuousConvMode = DISABLE;//Continuous conversion: input signal is sampled more than once
	ADC_InitStruct.ADC_ExternalTrigConv = 0;
	ADC_InitStruct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;//Data converted will be shifted to right
	ADC_InitStruct.ADC_NbrOfConversion = 14;
	ADC_Init(ADC1, &ADC_InitStruct);//Initialize ADC with the configuration

	/* Select the channels to be read from **************************************/
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_480Cycles);//PA1
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 2, ADC_SampleTime_480Cycles);//PA2
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 3, ADC_SampleTime_480Cycles);//PA3
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 4, ADC_SampleTime_480Cycles);//PA4
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 5, ADC_SampleTime_480Cycles);//PA5
	ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 6, ADC_SampleTime_480Cycles);//PA6
	ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 7, ADC_SampleTime_480Cycles);//PA7
	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 8, ADC_SampleTime_480Cycles);//PB0
	ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 9, ADC_SampleTime_480Cycles);//PB1
	//ADC_RegularChannelConfig(ADC3, ADC_Channel_10, 1, ADC_SampleTime_480Cycles);//PC0 -- cant use, connected to some LED in discovery board
	ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 10, ADC_SampleTime_480Cycles);//PC1
	ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 11, ADC_SampleTime_480Cycles);//PC2
	ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 12, ADC_SampleTime_480Cycles);//PC3
	ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 13, ADC_SampleTime_480Cycles);//PC4
	ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 14, ADC_SampleTime_480Cycles);//PC5

	/* Enable DMA request after last transfer (Single-ADC mode) */
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
	/* Enable ADC2 DMA */
	ADC_DMACmd(ADC1, ENABLE);
	/* Enable ADC2 */
	ADC_Cmd(ADC1, ENABLE);
}

void Config_Wakeup_INT(void)
{
	GPIO_InitTypeDef   GPIO_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;

	/* Enable GPIOA clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	/* Enable SYSCFG clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	/* Configure PA0 pin as input floating */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Connect EXTI Line0 to PA0 pin */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);

	/* Configure EXTI Line0 */
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Enable and set EXTI Line0 Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable WKUP pin  */
	PWR_WakeUpPinCmd(ENABLE);

}

void StartSleep(void){
	NVIC_InitTypeDef   NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;

	EXTI_InitStructure.EXTI_Line = EXTI_Line22; /*!< External interrupt line 22 Connected to the RTC Wakeup event */
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = RTC_WKUP_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	if(RTC_WakeUpCmd(DISABLE)==0){
		UB_Uart_SendString(COM2, "Error disabling WakeUp counter.", LFCR);
		while(1);
	}
	Delay(0xFFFF);

	RTC_WakeUpClockConfig(RTC_WakeUpClock_CK_SPRE_16bits);

	RTC_ClearFlag(RTC_FLAG_WUTF|RTC_FLAG_ALRBF|RTC_FLAG_ALRAF);

	// from AN3371 rev5:
	// If RTC clock PREDIV_A = div128 & PREDIV_S = div8192 then resolution is 32seconds
	// Already configured during RTC Init
	// WUTR = 0 -> 32 sec
	// WTR = 6f -> 3584 sec ~ 1hour
	RTC_SetWakeUpCounter(0x006F);

	if(RTC_WakeUpCmd(ENABLE)==0){
		UB_Uart_SendString(COM2, "Error enabling WakeUp counter.", LFCR);
		while(1);
	}

	/* Request to enter STANDBY mode (Wake Up flag is cleared in PWR_EnterSTANDBYMode function) */
	PWR_EnterSTANDBYMode();
}

void PrepareSleepMode(void){
	//disable other non relevant interrupts and clocks

	GPIO_InitTypeDef GPIO_InitStructure;
	/* Disable ADC3, DMA2 and GPIO clocks ****************************************/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, DISABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, DISABLE);//ADC3 is connected to the APB2 peripheral bus

	//disable I2C
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, DISABLE);
	I2C_Cmd(I2C1, DISABLE);

	DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, DISABLE);
	DMA_Cmd(DMA2_Stream0, DISABLE);

	/* DISABLE ADC3 DMA */
	ADC_DMACmd(ADC3, DISABLE);
	/* DISABLE ADC3 */
	ADC_Cmd(ADC3, DISABLE);

	//Disable USART
	USART_Cmd(USART2, DISABLE);
    USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, DISABLE);

    GPIO_LowPower_Config();

}

void GPIO_LowPower_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  /* Configure all GPIO port pins in Analog Input mode (floating input trigger OFF) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;

  GPIOA->MODER   = 0xFFFFFFFF;
  GPIOB->MODER   = 0xFFFFFFFF;
  GPIOC->MODER   = 0xFFFFFFFF;
  GPIOD->MODER   = 0xFFFFFFFF;
  GPIOE->MODER   = 0xFFFFFFFF;
  GPIOH->MODER   = 0xFFFFFFFF;

  GPIOA->PUPDR=0x0;
  GPIOB->PUPDR=0x0;
  GPIOC->PUPDR=0x0;
  GPIOD->PUPDR=0x0;
  GPIOE->PUPDR=0x0;
  GPIOH->PUPDR=0x0;

  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  GPIO_Init(GPIOE, &GPIO_InitStructure);

  //Disable GPIO clocks
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, DISABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, DISABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, DISABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, DISABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, DISABLE);

}

void PrepareRunMode(void){
		//initialize HW after sleep mode
		initHW();
}

/**
  * @brief  Configures the RTC.
  * @param  None
  * @retval None
  */
void RTC_Config(void)
{

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

  /* Allow access to BKP Domain */
  PWR_BackupAccessCmd(ENABLE);

  /* Clear Wakeup flag */
  PWR_ClearFlag(PWR_FLAG_WU);

  /* Check if the StandBy flag is set */
  if (PWR_GetFlagStatus(PWR_FLAG_SB) != RESET)
  {
    /* Clear StandBy flag */
    PWR_ClearFlag(PWR_FLAG_SB);

    /* Wait for RTC APB registers synchronisation (needed after start-up from Reset)*/
    RTC_WaitForSynchro();
    /* No need to configure the RTC as the RTC config(clock source, enable,
       prescaler,...) are kept after wake-up from STANDBY */
  }
  else
  {
    /* RTC Configuration ******************************************************/
    /* Reset Backup Domain */
    RCC_BackupResetCmd(ENABLE);
    RCC_BackupResetCmd(DISABLE);

/* The RTC Clock may varies due to LSI frequency dispersion. */
    /* Enable the LSI OSC */
    RCC_LSICmd(ENABLE);

    /* Wait till LSI is ready */
    while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
    {
    }

    /* Select the RTC Clock Source */
    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);

    /* Enable the RTC Clock */
    RCC_RTCCLKCmd(ENABLE);

    /* Wait for RTC APB registers synchronisation (needed after start-up from Reset)*/
    RTC_WaitForSynchro();

    /* Set the RTC time base to 1s */
    RTC_InitStructure.RTC_HourFormat = RTC_HourFormat_24;
    RTC_InitStructure.RTC_AsynchPrediv = 127;
    RTC_InitStructure.RTC_SynchPrediv = 8191;

    if (RTC_Init(&RTC_InitStructure) == ERROR)
    {
      /* Turn on LED3 */

      STM_EVAL_LEDOn(LED3);
      UB_Uart_SendString(COM2, "Error RTC Init. Blocked.", LFCR);
      /* User can add here some code to deal with this error */
      while(1);
    }

    /* Set the time to 01h 00mn 00s AM */
    RTC_TimeStructure.RTC_H12     = RTC_H12_AM;
    RTC_TimeStructure.RTC_Hours   = 0x01;
    RTC_TimeStructure.RTC_Minutes = 0x00;
    RTC_TimeStructure.RTC_Seconds = 0x00;

    if(RTC_SetTime(RTC_Format_BCD, &RTC_TimeStructure) == ERROR)
    {
      /* Turn on LED3 */
      STM_EVAL_LEDOn(LED3);
      UB_Uart_SendString(COM2, "Error RTC Set time. Blocked.", LFCR);
      /* User can add here some code to deal with this error */
      while(1);
    }
  }

  /* Clear RTC Alarm Flag */
  RTC_ClearFlag(RTC_FLAG_ALRAF);
}

/* Disable the Alarm A */
//RTC_AlarmCmd(RTC_Alarm_A, DISABLE);
//RTC_AlarmCmd(RTC_Alarm_B, DISABLE);
//	RTC_GetTime(RTC_Format_BCD, &RTC_TimeStructure);
//	/* Set the alarm to current time + 5s */
//	RTC_AlarmStructure.RTC_AlarmTime.RTC_H12     = RTC_H12_AM;
//	RTC_AlarmStructure.RTC_AlarmTime.RTC_Hours   = RTC_TimeStructure.RTC_Hours;
//	RTC_AlarmStructure.RTC_AlarmTime.RTC_Minutes = RTC_TimeStructure.RTC_Minutes+0x01;
//	RTC_AlarmStructure.RTC_AlarmTime.RTC_Seconds = RTC_TimeStructure.RTC_Seconds;
//	//RTC_AlarmStructure.RTC_AlarmTime.RTC_Seconds = (RTC_TimeStructure.RTC_Seconds + 0x5) % 60;
//	RTC_AlarmStructure.RTC_AlarmDateWeekDay = 0x31;
//	RTC_AlarmStructure.RTC_AlarmDateWeekDaySel = RTC_AlarmDateWeekDaySel_Date;
//	RTC_AlarmStructure.RTC_AlarmMask = RTC_AlarmMask_DateWeekDay;
//	RTC_SetAlarm(RTC_Format_BCD, RTC_Alarm_A, &RTC_AlarmStructure);
//
//	/* Enable RTC Alarm A Interrupt: this Interrupt will wake-up the system from
//			       STANDBY mode (RTC Alarm IT not enabled in NVIC) */
//	RTC_ITConfig(RTC_IT_ALRA, ENABLE);
//
//	/* Enable the Alarm A */
//	RTC_AlarmCmd(RTC_Alarm_A, ENABLE);
//
//	/* Clear RTC Alarm Flag */



/**
  * @brief  Configures the SysTick to generate an interrupt each 250 ms.
  * @param  None
  * @retval None
  */

void SysTick_Configuration(void)
{
  /* SysTick interrupt each 250 ms */
  if (SysTick_Config((SystemCoreClock/8) / 4))
  {
    /* Capture error */
    while (1);
  }

  /* Select AHB clock(HCLK) divided by 8 as SysTick clock source */
  SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);

  /* Set SysTick Preemption Priority to 1 */
  NVIC_SetPriority(SysTick_IRQn, 0x04);
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif
