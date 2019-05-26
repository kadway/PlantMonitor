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
			NULL                              /* Task handle */
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
			NULL                              /* Task handle */
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

	timerHndl1Sec = xTimerCreate( "timer1Sec", /* name */
			pdMS_TO_TICKS(5000), 			   /* period/time */
			pdTRUE, 						   /* auto reload */
			(void*)0, 					       /* timer ID */
			vTimerCallback1SecExpired); 	   /* callback */

	if (timerHndl1Sec==NULL) {
		for(;;); /* failure! */
		UB_Uart_SendString(COM2,"Timer failure",LFCR);
	}
  if (xTimerStart(timerHndl1Sec, 0)!=pdPASS) {
	  for(;;); /* failure!?! */
	  UB_Uart_SendString(COM2,"Failure",LFCR);
  }

}


// Task functions

/**
 * TASK 1: Task to read data from sensors
 */
void PoolSensors(void *pvParameters){
	uint16_t* ADC_pointer=NULL;

	xSemaphore = xSemaphoreCreateMutex();
	//start first conversion
	//ADC_SoftwareStartConv(ADC3);
	ADC_pointer=getSensorValues();

	for (;;) {
		UB_Uart_SendString(COM2,"PoolSensors",LFCR);
		//UB_Uart_SendString(COM2,"sensorTask suspended",LFCR);
		//vTaskSuspend(sensorTaskHndl); //suspend itself
		//	UB_Uart_SendString(COM2,"sensorTask processing data",LFCR);

		if(dataReady){
			if( xSemaphoreTake( xSemaphore, ( TickType_t ) 100 ) == pdTRUE ){

				dataReady = 0;
				for(i=0; i<4; i++){
					ADC_pointer[i] = (uint16_t) (( (uint32_t)ADC_pointer[i] * 3000) / 4096);
				}
				//				for(i=0; i<4; i++){
				//					sprintf(sAdcValue, "Channel %d, value: %d mV", i, ADC3ConvertedValue[i]);
				//					UB_Uart_SendString(COM2, sAdcValue, LFCR);
				//				}
				UB_Uart_SendString(COM2, "Adc ready", LFCR);
				xSemaphoreGive( xSemaphore );
			}
		}
		vTaskDelay(3 * configTICK_RATE_HZ );
	}
}

/**
 * TASK 2: Task to actuate solenoid valves and water pump
 */

void WaterPlants(void *pvParameters){
	//TODO: read in the measured values and decide on the actuation of the water pump
	//      create a timer to control the watering time

	for (;;) {
		UB_Uart_SendString(COM2,"WaterPlants",LFCR);
		//GPIO_ToggleBits(GPIOD, GPIO_Pin_12);

		vTaskDelay(3*configTICK_RATE_HZ);
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
	while (1) {
		UB_Uart_SendString(COM2,"Sleep alarm task",LFCR);
		vTaskDelay(5*configTICK_RATE_HZ);
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
			UB_Uart_SendString(COM2,"Button pressed",LFCR);
		}
	}
}



//Timer functions

static void vTimerCallback1SecExpired(xTimerHandle pxTimer) {
	char buf_time[45];
	char buf_alarm[25];
	char buf_temp[25];
	int8_t integer;
	uint8_t fractional;
	ds3231_status status;

	GPIOE -> ODR ^= GPIO_Pin_2;
	sprintf(sCounter, "Timer %d", counter);
	counter++;
	UB_Uart_SendString(COM2,sCounter,LFCR);

	if(~status.BSY){
		//get current time
		time_pt = rtc_get_time();
		sprintf(buf_time, "Hora %d:%d:%d dia %d do mes %d", time_pt->hour, time_pt->min, time_pt->sec, time_pt->mday, time_pt->mon);
		UB_Uart_SendString(COM2, buf_time, LFCR);

		//rtc_read_byte((uint8_t*) &status, STATUS_ADDR);

		//get current temperature
		readTemperatureRTC(&integer, &fractional);
		sprintf(buf_temp, "Temperatura: %d.%d ºC", integer, fractional);
		UB_Uart_SendString(COM2, buf_temp, LFCR);

		//rtc_reset_alarm();
		time_pt->sec = time_pt->sec + 3;
		Delay(0x3FFF);
		rtc_set_alarm(time_pt);
		//rtc_set_alarm_s(uint8_t hour, uint8_t min, uint8_t sec);
		//UB_Uart_SendString(COM2, "Check alarm", LFCR);
		Delay(0x3FFF);
		time_pt = rtc_get_alarm();
		sprintf(buf_alarm, "Alarme set %d:%d:%d", time_pt->hour, time_pt->min, time_pt->sec);
		UB_Uart_SendString(COM2, buf_alarm, LFCR);
	    /* Enter Stop Mode */
		Delay(0x3FFF);
	    PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);
	    //reconfigure clock system
	    SystemInit();
	}
	else
	{
		UB_Uart_SendString(COM2, "Device busy", LFCR);
	}

	if( xSemaphore != NULL ){
		if( xSemaphoreTake( xSemaphore, ( TickType_t ) 100 ) == pdTRUE ){
			ADC_SoftwareStartConv(ADC3);
			xSemaphoreGive( xSemaphore );
			}
		 }
}
