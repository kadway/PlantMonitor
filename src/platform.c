/*
 * platform.c
 *
 *  Created on: May 8, 2019
 *      Author: johny
 */

#include "platform.h"


void initHW()
{

	SystemInit(); // Clock init

	// Init UART
	// Com2 115200 Baud
	UB_Uart_Init();


	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure2;

	// Init LED
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	// Init PushButton
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_InitStructure2.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure2.GPIO_Pin =  GPIO_Pin_0;
	GPIO_InitStructure2.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure2.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure2.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure2);

	//Configure and start I2c
	I2C_Config();

	rtc_init();

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

	I2C_InitStruct.I2C_ClockSpeed = 100000; // 100kHz
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;   // I2C mode
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;   // 50% duty cycle --> standard
	I2C_InitStruct.I2C_OwnAddress1 = 0x0;   // own address, not relevant in master mode
	I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;   // Enable acknowledge when reading
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // set address length
	I2C_Init(I2C1, &I2C_InitStruct);   // init I2C1

    I2C_Cmd(I2C1, ENABLE);

}


void ADC_Config(uint16_t *ADC3ConvertedValue)
{

	ADC_InitTypeDef       ADC_InitStruct;
	ADC_CommonInitTypeDef ADC_CommonInitStruct;
	DMA_InitTypeDef       DMA_InitStruct;
	GPIO_InitTypeDef      GPIO_InitStruct;
	NVIC_InitTypeDef   NVIC_InitStructure;

	ADC_DeInit();

	/* Enable ADC3, DMA2 and GPIO clocks ****************************************/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);//ADC3 is connected to the APB2 peripheral bus

	/* DMA2 Stream0 channel0 configuration **************************************/
	DMA_InitStruct.DMA_Channel = DMA_Channel_2;
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&ADC3->DR;//ADC3's data register
	DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)ADC3ConvertedValue;
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStruct.DMA_BufferSize = 4;
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

	DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, ENABLE);
	  //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	  //NVIC_SetPriorityGrouping( 0 );
    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);


	/* Configure GPIO pins ******************************************************/
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;// PC1, PC2, PC3
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;//The pins are configured in analog mode
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL ;//We don't need any pull up or pull down
	GPIO_Init(GPIOC, &GPIO_InitStruct);//Initialize GPIOC pins with the configuration
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;//PA1
	GPIO_Init(GPIOA, &GPIO_InitStruct);//Initialize GPIOA pins with the configuration

	/* ADC Common Init **********************************************************/
	ADC_CommonInitStruct.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStruct.ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_CommonInitStruct.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStruct.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStruct);

	/* ADC3 Init ****************************************************************/
	ADC_DeInit();
	ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;//Input voltage is converted into a 12bit int (max 4095)
	ADC_InitStruct.ADC_ScanConvMode = ENABLE;//The scan is configured in multiple channels
	ADC_InitStruct.ADC_ContinuousConvMode = DISABLE;//Continuous conversion: input signal is sampled more than once
	ADC_InitStruct.ADC_ExternalTrigConv = 0;
	ADC_InitStruct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;//Data converted will be shifted to right
	ADC_InitStruct.ADC_NbrOfConversion = 4;
	ADC_Init(ADC3, &ADC_InitStruct);//Initialize ADC with the configuration

	/* Select the channels to be read from **************************************/
	//ADC_RegularChannelConfig(ADC3, ADC_Channel_10, 1, ADC_SampleTime_480Cycles);//PC0 -- cant use connected to some LED in discovery board
	ADC_RegularChannelConfig(ADC3, ADC_Channel_11, 1, ADC_SampleTime_480Cycles);//PC1
	ADC_RegularChannelConfig(ADC3, ADC_Channel_12, 2, ADC_SampleTime_480Cycles);//PC2
	ADC_RegularChannelConfig(ADC3, ADC_Channel_13, 3, ADC_SampleTime_480Cycles);//PC3
	ADC_RegularChannelConfig(ADC3, ADC_Channel_1,  4, ADC_SampleTime_480Cycles);//PA1

	/* Enable DMA request after last transfer (Single-ADC mode) */
	ADC_DMARequestAfterLastTransferCmd(ADC3, ENABLE);

	/* Enable ADC3 DMA */
	ADC_DMACmd(ADC3, ENABLE);

	/* Enable ADC3 */
	ADC_Cmd(ADC3, ENABLE);

	//ADC_SoftwareStartConv(ADC3);

}
