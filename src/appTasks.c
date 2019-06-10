#include "appTasks.h"


/* Create tasks */
void createTasks(void){

	xTaskCreate(
			PoolSensors,                 /* Function pointer */
			"Task1",                   /* Task name - for debugging only*/
			configMINIMAL_STACK_SIZE,         /* Stack depth in words */
			(void*) NULL,                     /* Pointer to tasks arguments (parameter) */
			tskIDLE_PRIORITY + 2UL,           /* Task priority*/
			&sensorTaskHndl                   /* Task handle */
	);

	xTaskCreate(
			WaterPlants,                 /* Function pointer */
			"Task2",                          /* Task name - for debugging only*/
			configMINIMAL_STACK_SIZE,         /* Stack depth in words */
			(void*) NULL,                     /* Pointer to tasks arguments (parameter) */
			tskIDLE_PRIORITY + 2UL,           /* Task priority*/
			&waterTaskHndl                             /* Task handle */
	);

	xTaskCreate(
			SendDataOut_IPC,
			"Task3",
			configMINIMAL_STACK_SIZE,
			(void*) NULL,
			tskIDLE_PRIORITY + 2UL,
			NULL);

	xTaskCreate(
			SleepAlarm,                 /* Function pointer */
			"Task4",                          /* Task name - for debugging only*/
			configMINIMAL_STACK_SIZE,         /* Stack depth in words */
			(void*) NULL,                     /* Pointer to tasks arguments (parameter) */
			tskIDLE_PRIORITY + 2UL,           /* Task priority*/
			&sleepTaskHndl                              /* Task handle */
	);

	xTaskCreate(
			DetectButtonPress,
			"Task5",
			configMINIMAL_STACK_SIZE,
			(void*) NULL,
			tskIDLE_PRIORITY + 2UL,
			NULL);

	//create semaphore
	xSemaphore = xSemaphoreCreateBinary();

	/* Create IPC variables */
	pbq = xQueueCreate(10, sizeof(int));
	if (pbq == 0) {
		while(1); /* fatal error */
	}

}

//if( xSemaphore != NULL ){
//	if( xSemaphoreTake( xSemaphore, ( TickType_t ) 100 ) == pdTRUE ){
//		ADC_SoftwareStartConv(ADC3);
//		xSemaphoreGive( xSemaphore );
//	}
//}

// Task functions

/**
 * TASK 1: Task to read data from sensors
 */
void PoolSensors(void *pvParameters){


	xSemaphore = xSemaphoreCreateMutex();


	for (;;) {
		//UB_Uart_SendString(COM3,"PoolSensors",LFCR);

		if(!dataReady){
			//UB_Uart_SendString(COM3,"Start ADC conversion",LFCR);
			ADC_SoftwareStartConv(ADC3);
		}
		//UB_Uart_SendString(COM3,"sensorTask suspended",LFCR);
		//vTaskSuspend(sensorTaskHndl); //suspend itself
		//	UB_Uart_SendString(COM3,"sensorTask processing data",LFCR);

		if(dataReady){
			//if( xSemaphoreTake( xSemaphore, ( TickType_t ) 100 ) == pdTRUE ){
				dataReady = 0;
				//UB_Uart_SendString(COM3, "Adc ready", LFCR);
				//xSemaphoreGive( xSemaphore );
				vTaskResume(waterTaskHndl);
				vTaskSuspend(NULL);
			//}
		}
		//delay wait for ADC conversion finished
		vTaskDelay(2 * configTICK_RATE_HZ );
	}
}

/**
 * TASK 2: Task to actuate solenoid valves and water pump
 */

void WaterPlants(void *pvParameters){
	//TODO: read in the measured values and decide on the actuation of the water pump
	//      create a timer to control the watering time

	uint16_t* ADC_pointer=NULL;
	ADC_pointer=getSensorValues();
	char buf_temp[25];
	int8_t integer;
	uint8_t fractional;

	for (;;) {
		//suspend itself wait for pooling sensor task to resume it
		vTaskSuspend(NULL);
		//UB_Uart_SendString(COM3,"WaterPlants",LFCR);
		readTemperatureRTC(&integer, &fractional);
		sprintf(buf_temp, "Temp: %d.%d ºC", integer, fractional);
		UB_Uart_SendString(COM3, buf_temp, LFCR);

		for(i=0; i<4; i++){
			ADC_pointer[i] = (uint16_t) (( (uint32_t)ADC_pointer[i] * 3000) / 4096);
		}
		//for(i=0; i<4; i++){
		//	sprintf(sAdcValue, "Channel %d, value: %d mV", i, ADC_pointer[i]);
		//	UB_Uart_SendString(COM3, sAdcValue, LFCR);
		//}

		timerHndl1Sec = xTimerCreate( "timer1Sec", /* name */
				pdMS_TO_TICKS(1000), 			   /* period/time */
				pdFALSE, 						   /* auto reload */
				(void*)0, 					       /* timer ID */
				vTimerCallback1SecExpired); 	   /* callback */

		if (timerHndl1Sec==NULL) {
			for(;;); /* failure! */
			UB_Uart_SendString(COM3,"Timer failure",LFCR);
		}
		if (xTimerStart(timerHndl1Sec, 0)!=pdPASS) {
			for(;;); /* failure!?! */
			UB_Uart_SendString(COM3,"Failure",LFCR);
		}

		//UB_Uart_SendString(COM3,"Watering... Wait for timer",LFCR);
		//suspend itself timer should take over
		vTaskSuspend(NULL);
	}
}


/**
 * TASK 3: Receive data via Inter-Process Communication (IPC) and send it out
 *
 */

void SendDataOut_IPC(void *pvParameters) {
	//TODO: Receive monitored values from sensors and send them over the air to be processed elsewhere (with NRF24L?)
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

/**
 * TASK 5: Detect Button Press
 * 			And Signal Event via Inter-Process Communication (IPC)
 */
void SleepAlarm(void *pvParameters){
	char buf_time[45];
	struct tm *time_pt=NULL;

	while (1) {
		//suspend itself
		vTaskSuspend(NULL);
		//get current time
		time_pt = rtc_get_time();
		sprintf(buf_time, "Going to sleep at %d:%d:%d - %d/%d", time_pt->hour, time_pt->min, time_pt->sec, time_pt->mday, time_pt->mon);
		UB_Uart_SendString(COM3, buf_time, LFCR);
		Delay(0xFFFF);
		PrepareSleepMode();
		StartSleep();

//		time_pt->min = time_pt->min + 1;
//
//		Delay(0x3FFF);
//		rtc_set_alarm(time_pt);
//		Delay(0x3FFF);
//		time_pt = rtc_get_alarm();
//		sprintf(buf_alarm, "Alarm set %d:%d:%d", time_pt->hour, time_pt->min, time_pt->sec);
//		UB_Uart_SendString(COM3, buf_alarm, LFCR);
//		Delay(0x3FFF);
//
//		//disable not needed clocks and interrupts
//		//PrepareSleepMode();
//
//		/* Enter Stop Mode */
//		//PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFE);
//
//		//reconfigure system
//		//PrepareRunMode();
//		while(alarm_not_fired){
//			UB_Uart_SendString(COM3, "wait alarm", LFCR);
//			//PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFE);
//			vTaskDelay(5*configTICK_RATE_HZ);
//
//		}
//		/* Toggle LED */
//		//GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
//		//time_pt = rtc_get_time();
//		//sprintf(buf_time, "Alarm fired: %d:%d:%d", time_pt->hour, time_pt->min, time_pt->sec);
//		//UB_Uart_SendString(COM3, buf_time, LFCR);
//		UB_Uart_SendString(COM3, "resume Sensor pooling", LFCR);
//		//resume pool sensor task
//		vTaskResume(sensorTaskHndl);
	}
}


/**
 * TASK 5: Detect Button Press
 * 			And Signal Event via Inter-Process Communication (IPC)
 */
void DetectButtonPress(void *pvParameters){
	//TODO: still left over from original code. Decide if a button press is relevant for the application.
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
			UB_Uart_SendString(COM3,"Button pressed",LFCR);
		}
	}
}



//Timer functions

static void vTimerCallback1SecExpired(xTimerHandle pxTimer) {
	char sCounter[50];

	//sprintf(sCounter, "Timer %d expired. Resume sleep task.", counter);
	counter++;
	//UB_Uart_SendString(COM3,sCounter,LFCR);
	vTaskResume(sleepTaskHndl);
}
