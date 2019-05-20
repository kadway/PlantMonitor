#include <stdio.h>
#include <stdlib.h>

/* Board includes */
#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h"
/*
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_dma.h"*/

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
/* UART library include */
#include "stm32_ub_uart.h"

#include "platform.h"
#include "helpers.h"


#define BLOCK_
// tasks Prototypes
void ToggleLED_Timer(void*);
void DetectButtonPress(void*);
void ToggleLED_IPC(void*);
void GetSensorValues(void*);

// statically allocated
struct tm *time_pt=NULL;

/*Task handles*/
TaskHandle_t sensorTaskHndl = NULL;

/*Global Variables*/
char sAdcValue[24];
char sCounter[24];
uint16_t ADC3ConvertedValue[5] = {0,0,0,0,0};

xQueueHandle pbq;
xTimerHandle timerHndl1Sec;
SemaphoreHandle_t xSemaphore = NULL;

uint16_t i = 0;
uint16_t counter = 0;
int8_t dataReady = 0;

//Timer Handler
static void vTimerCallback1SecExpired(xTimerHandle pxTimer) {
	GPIOE -> ODR ^= GPIO_Pin_2;
	sprintf(sCounter, "Timer %d", counter);
	counter++;
	UB_Uart_SendString(COM2,sCounter,LFCR);

	if( xSemaphore != NULL ){
		if( xSemaphoreTake( xSemaphore, ( TickType_t ) 100 ) == pdTRUE ){
			ADC_SoftwareStartConv(ADC3);
			xSemaphoreGive( xSemaphore );
		}
	 }
}

int main(void){

	initHW();
	// ADC config
	ADC_Config(ADC3ConvertedValue);
	//debug

	UB_Uart_SendString(COM2,"HW initialized",LFCR);
	GPIO_SetBits(GPIOD, GPIO_Pin_12);
	//ADC_SoftwareStartConv(ADC3);
	//create semaphore
	 xSemaphore = xSemaphoreCreateBinary();

	/* Create IPC variables */
	pbq = xQueueCreate(10, sizeof(int));
	if (pbq == 0) {
		while(1); /* fatal error */
	}

	timerHndl1Sec = xTimerCreate( "timer1Sec", /* name */
			pdMS_TO_TICKS(4000), 			   /* period/time */
			pdTRUE, 						   /* auto reload */
			(void*)0, 					       /* timer ID */
			vTimerCallback1SecExpired); 	   /* callback */

	if (timerHndl1Sec==NULL) {
		for(;;); /* failure! */
		UB_Uart_SendString(COM2,"Failure creating timer",LFCR);
	}

	/* Create tasks */
	xTaskCreate(
			GetSensorValues,                 /* Function pointer */
			"sensorTask",                   /* Task name - for debugging only*/
			configMINIMAL_STACK_SIZE,         /* Stack depth in words */
			(void*) NULL,                     /* Pointer to tasks arguments (parameter) */
			tskIDLE_PRIORITY + 2UL,           /* Task priority*/
			&sensorTaskHndl                   /* Task handle */
	);

	xTaskCreate(
			ToggleLED_Timer,                 /* Function pointer */
			"Task1",                          /* Task name - for debugging only*/
			configMINIMAL_STACK_SIZE,         /* Stack depth in words */
			(void*) NULL,                     /* Pointer to tasks arguments (parameter) */
			tskIDLE_PRIORITY + 2UL,           /* Task priority*/
			NULL                              /* Task handle */
	);

	xTaskCreate(
			DetectButtonPress,
			"Task2",
			configMINIMAL_STACK_SIZE,
		  (void*) NULL,
		  tskIDLE_PRIORITY + 2UL,
		  NULL);

  xTaskCreate(
		  ToggleLED_IPC,
		  "Task3",
		  configMINIMAL_STACK_SIZE,
		  (void*) NULL,
		  tskIDLE_PRIORITY + 2UL,
		  NULL);
  
  if (xTimerStart(timerHndl1Sec, 0)!=pdPASS) {
	  for(;;); /* failure!?! */
  }


  /* Start the RTOS Scheduler */
  vTaskStartScheduler();

  /* HALT */
  while(1);

}

// Task to read data from sensors
void GetSensorValues(void *pvParameters){
	xSemaphore = xSemaphoreCreateMutex();
	//start first conversion
	ADC_SoftwareStartConv(ADC3);

	for (;;) {

		//UB_Uart_SendString(COM2,"sensorTask suspended",LFCR);
		//vTaskSuspend(sensorTaskHndl); //suspend itself
		//	UB_Uart_SendString(COM2,"sensorTask processing data",LFCR);

		if(dataReady){
			if( xSemaphoreTake( xSemaphore, ( TickType_t ) 100 ) == pdTRUE ){

				dataReady = 0;
				for(i=0; i<4; i++){
					ADC3ConvertedValue[i] = (uint16_t) (( (uint32_t)ADC3ConvertedValue[i] * 3000) / 4096);
				}
				for(i=0; i<4; i++){
					sprintf(sAdcValue, "Channel %d, value: %d mV", i, ADC3ConvertedValue[i]);
					UB_Uart_SendString(COM2, sAdcValue, LFCR);
				}
				xSemaphoreGive( xSemaphore );
			}
		}
		vTaskDelay(1000 * configTICK_RATE_HZ / 1000 );
	}
}

/**
 * TASK 1: Toggle LED via RTOS Timer
 */
void ToggleLED_Timer(void *pvParameters){
	 for (;;) {
		// Perform action here.
		char buffer[20];
		UB_Uart_SendString(COM2, "ds3231 task", LFCR);
		GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
		time_pt = rtc_get_time();
		sprintf(buffer, "Time is %d:%d:%d", i, time_pt->hour, time_pt->min, time_pt->sec);
		UB_Uart_SendString(COM2, buffer, LFCR);

		vTaskDelay(1000 * configTICK_RATE_HZ / 1000);
	}
}

/**
 * TASK 2: Detect Button Press
 * 			And Signal Event via Inter-Process Communication (IPC)
 */
void DetectButtonPress(void *pvParameters){

	int sig = 1;

	while (1) {
		/* Detect Button Press  */
		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)>0) {
			while(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)>0){
				vTaskDelay(pdMS_TO_TICKS(500)); /* Button Debounce Delay */
			}
			while(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)==0){
				vTaskDelay(pdMS_TO_TICKS(500)); /* Button Debounce Delay */
			}

			xQueueSendToBack(pbq, &sig, 0); /* Send Message */
			UB_Uart_SendString(COM2,"Button pressed",LFCR);
		}
	}
}

/**
 * TASK 3: Toggle LED via Inter-Process Communication (IPC)
 *
 */

void ToggleLED_IPC(void *pvParameters) {
  
  int sig;
  portBASE_TYPE status;
  
  while (1) {
    status = xQueueReceive(pbq, &sig, portMAX_DELAY); /* Receive Message */
    												  /* portMAX_DELAY blocks task indefinitely if queue is empty */
    if(status == pdTRUE) {
      GPIO_ToggleBits(GPIOD,GPIO_Pin_14);
    }
  }
}


void DMA2_Stream0_IRQHandler(void) {

	if(DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0)) {

		//temporary solution for signaling that data is ready
		dataReady = 1;

		//resume the task that prints out the data
		// NOT WORKING
		/*BaseType_t xYieldRequired;
		xYieldRequired = pdFALSE;
		xYieldRequired = xTaskResumeFromISR(sensorTaskHndl);
		portYIELD_FROM_ISR(xYieldRequired);*/
		/*BaseType_t xHigherPriorityTaskWoken;
		xHigherPriorityTaskWoken = pdFALSE;
		vTaskNotifyGiveFromISR( sensorTaskHndl, &xHigherPriorityTaskWoken );
		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );*/
	}

	DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);		//clear all the interrupts
	DMA_ClearFlag(DMA2_Stream0, DMA_FLAG_TCIF0);  		//make sure flags are clear
}




