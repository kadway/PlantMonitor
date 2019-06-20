#include "appTasks.h"

Pot_t allPots[] = {
			{POT1, MOIST, DEFAULT_WATER_TIME, POT1_THRESHOLD,POT1_NUM_SENS,
				{POT1_WATER_PUMP_PORT,POT1_WATER_PUMP_PIN},
				{POT1_SOLENOID_PORT,POT1_SOLENOID_PIN},
					{{POT1_SENSOR1_SUPPLY_PORT,POT1_SENSOR1_SUPPLY_PIN},{POT1_SENSOR2_SUPPLY_PORT,POT1_SENSOR2_SUPPLY_PIN},
					{POT1_SENSOR3_SUPPLY_PORT,POT1_SENSOR3_SUPPLY_PIN},{POT1_SENSOR4_SUPPLY_PORT,POT1_SENSOR4_SUPPLY_PIN}}},

			{POT2, MOIST, DEFAULT_WATER_TIME, POT2_THRESHOLD,POT2_NUM_SENS,
				{POT2_WATER_PUMP_PORT,POT2_WATER_PUMP_PIN},
				{POT2_SOLENOID_PORT,POT2_SOLENOID_PIN},
					{{POT2_SENSOR1_SUPPLY_PORT,POT2_SENSOR1_SUPPLY_PIN},{POT2_SENSOR2_SUPPLY_PORT,POT2_SENSOR2_SUPPLY_PIN},
					{POT2_SENSOR3_SUPPLY_PORT,POT2_SENSOR3_SUPPLY_PIN},{POT2_SENSOR4_SUPPLY_PORT,POT2_SENSOR4_SUPPLY_PIN}}},

			{POT3, MOIST, DEFAULT_WATER_TIME, POT3_THRESHOLD,POT3_NUM_SENS,
				{POT3_WATER_PUMP_PORT,POT3_WATER_PUMP_PIN},
				{POT3_SOLENOID_PORT,POT3_SOLENOID_PIN},
					{{POT3_SENSOR1_SUPPLY_PORT,POT3_SENSOR1_SUPPLY_PIN},{POT3_SENSOR2_SUPPLY_PORT,POT3_SENSOR2_SUPPLY_PIN},
					{POT3_SENSOR3_SUPPLY_PORT,POT3_SENSOR3_SUPPLY_PIN},{POT3_SENSOR4_SUPPLY_PORT,POT3_SENSOR4_SUPPLY_PIN}}},

			{POT4, MOIST, DEFAULT_WATER_TIME, POT4_THRESHOLD,POT4_NUM_SENS,
				{POT4_WATER_PUMP_PORT,POT4_WATER_PUMP_PIN},
				{POT4_SOLENOID_PORT,POT4_SOLENOID_PIN},
					{{POT4_SENSOR1_SUPPLY_PORT,POT4_SENSOR1_SUPPLY_PIN},{POT4_SENSOR2_SUPPLY_PORT,POT4_SENSOR2_SUPPLY_PIN},
					{POT4_SENSOR3_SUPPLY_PORT,POT4_SENSOR3_SUPPLY_PIN},{POT4_SENSOR4_SUPPLY_PORT,POT4_SENSOR4_SUPPLY_PIN}}},

			{POT5, MOIST, DEFAULT_WATER_TIME, POT5_THRESHOLD,POT5_NUM_SENS,
				{POT5_WATER_PUMP_PORT,POT5_WATER_PUMP_PIN},
				{POT5_SOLENOID_PORT,POT5_SOLENOID_PIN},
					{{POT5_SENSOR1_SUPPLY_PORT,POT5_SENSOR1_SUPPLY_PIN},{POT5_SENSOR2_SUPPLY_PORT,POT5_SENSOR2_SUPPLY_PIN},
					{POT5_SENSOR3_SUPPLY_PORT,POT5_SENSOR3_SUPPLY_PIN},{POT5_SENSOR4_SUPPLY_PORT,POT5_SENSOR4_SUPPLY_PIN}}},

			{POT6, MOIST, DEFAULT_WATER_TIME, POT6_THRESHOLD,POT6_NUM_SENS,
				{POT6_WATER_PUMP_PORT,POT6_WATER_PUMP_PIN},
					{POT6_SOLENOID_PORT,POT6_SOLENOID_PIN},
					{{POT6_SENSOR1_SUPPLY_PORT,POT6_SENSOR1_SUPPLY_PIN},{POT6_SENSOR2_SUPPLY_PORT,POT6_SENSOR2_SUPPLY_PIN},
					{POT6_SENSOR3_SUPPLY_PORT,POT6_SENSOR3_SUPPLY_PIN},{POT6_SENSOR4_SUPPLY_PORT,POT6_SENSOR4_SUPPLY_PIN}}},
};

/* Create tasks */
void createTasks(void){

	xTaskCreate(
			PoolSensors,                 /* Function pointer */
			"Task1",                   /* Task name - for debugging only*/
			configMINIMAL_STACK_SIZE*4,         /* Stack depth in words */
			(void*) NULL,                     /* Pointer to tasks arguments (parameter) */
			tskIDLE_PRIORITY + 2UL,           /* Task priority*/
			&sensorTaskHndl                   /* Task handle */
	);

	xTaskCreate(
			WaterPlants,                 /* Function pointer */
			"Task2",                          /* Task name - for debugging only*/
			configMINIMAL_STACK_SIZE*100,         /* Stack depth in words */
			(void*) NULL,                     /* Pointer to tasks arguments (parameter) */
			tskIDLE_PRIORITY + 2UL,           /* Task priority*/
			&waterTaskHndl                             /* Task handle */
	);

//	xTaskCreate(
//			SendDataOut_IPC,
//			"Task3",
//			configMINIMAL_STACK_SIZE,
//			(void*) NULL,
//			tskIDLE_PRIORITY + 2UL,
//			NULL);

	xTaskCreate(
			SleepAlarm,                 /* Function pointer */
			"Task4",                          /* Task name - for debugging only*/
			configMINIMAL_STACK_SIZE,         /* Stack depth in words */
			(void*) NULL,                     /* Pointer to tasks arguments (parameter) */
			tskIDLE_PRIORITY + 2UL,           /* Task priority*/
			&sleepTaskHndl                              /* Task handle */
	);

//	xTaskCreate(
//			DetectButtonPress,
//			"Task5",
//			configMINIMAL_STACK_SIZE,
//			(void*) NULL,
//			tskIDLE_PRIORITY + 2UL,
//			NULL);

	//create semaphore
//	xSemaphore = xSemaphoreCreateBinary();
   //create a timer
	timerHndl1Sec = xTimerCreate( "timer1Sec", /* name */
			pdMS_TO_TICKS(DEFAULT_WATER_TIME), 	  /* period/time */
			pdFALSE, 						   /* auto reload */
			(void*)0, 					       /* timer ID */
			vTimerCallback1SecExpired); 	   /* callback */

	if (timerHndl1Sec==NULL) {
		UB_Uart_SendString(COM3,"Timer failure",LFCR);
	}

	/* Create IPC variables */
//	pbq = xQueueCreate(10, sizeof(int));
//	if (pbq == 0) {
//		while(1); /* fatal error */
//	}

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
	uint16_t i = 0;
	uint16_t j = 0;
	for (;;) {
		//UB_Uart_SendString(COM3,"PoolSensors",LFCR);

		if(!adcready){
			if(!ADC_GetSoftwareStartConvStatus(ADC1)){
				//Enable voltage for moisture sensors
				//for(i=0;i<NUM_POTS; i++){
					//for(j=0;j<allPots[i].num_sensors; j++){
					//	GPIO_SetBits(allPots[i].sens_sup->PORT, allPots[i].sens_sup->PIN);
					//}

				//}
				GPIO_SetBits(GPIOD, GPIOD_SENSORS_SUPPLY_PINS);
				GPIO_SetBits(GPIOE, GPIOE_SENSORS_SUPPLY_PINS);
				//start ADC conversions
				vTaskDelay(1 * configTICK_RATE_HZ );
				ADC_SoftwareStartConv(ADC1);
			}
		}
		if(adcready){
			//disable voltage for moisture sensors
//			for(i=0;i<NUM_POTS; i++){
//				for(j=0;j<allPots[i].num_sensors; j++){
//					GPIO_ResetBits(allPots[i].sens_sup->PORT, allPots[i].sens_sup->PIN);
//				}
//			}
			GPIO_ResetBits(GPIOD, GPIOD_SENSORS_SUPPLY_PINS);
			GPIO_ResetBits(GPIOE, GPIOE_SENSORS_SUPPLY_PINS);
			vTaskResume(waterTaskHndl);
			vTaskSuspend(NULL);
		}
		vTaskDelay(1 * configTICK_RATE_HZ );
	}
}

/**
 * TASK 2: Task to actuate solenoid valves and water pump
 */

void WaterPlants(void *pvParameters){
	//TODO: read in the measured values and decide on the actuation of the water pump
	//      create a timer to control the watering time

	uint16_t* ADC1Data=NULL;
	ADC1Data=getSensorValues();
	char buf_temp[25];
	int8_t integer;
	uint8_t fractional;
	uint16_t i = 0;
	int PotNum = 0;
	char buf[50];
	struct tm *time_pt=NULL;
	for (;;) {
		//suspend itself wait for pooling sensor task to resume it
		vTaskSuspend(NULL);
		//UB_Uart_SendString(COM3,"WaterPlants",LFCR);
		readTemperatureRTC(&integer, &fractional);
		sprintf(buf_temp, "Temp: %d.%d ºC", integer, fractional);
		UB_Uart_SendString(COM3, buf_temp, LFCR);

		//get respective voltage values for the adc conversions
		for(i=0; i<14; i++){
			ADC1Data[i] = (uint16_t) (( (uint32_t)ADC1Data[i] * 3000) / 4096);
		}

		//print out the results
		for(i=0; i<14; i++){
			sprintf(sAdcValue, "ADC1 pin %s, value: %d mV", ADC1_Pins[i], ADC1Data[i]);
			UB_Uart_SendString(COM3, sAdcValue, LFCR);
		}

		for (PotNum=0; PotNum<NUM_POTS; PotNum++){
			sprintf(buf, "Check pot %d", PotNum+1);
			UB_Uart_SendString(COM3, buf, LFCR);
			allPots[PotNum].status=CheckMoisture(PotNum);
			if(allPots[PotNum].status==DRY){
				WaterPot(PotNum, integer);
			}
		}


		//UB_Uart_SendString(COM3, "resume sleep", LFCR);
		//Delay(0xFFFF);
		vTaskResume(sleepTaskHndl);
		//suspend itself
		vTaskSuspend(NULL);
	}
}
uint8_t WaterPot(uint8_t pot, int8_t temperature){
		//TickType_t time_ms = 1000; //~ 30 seconds
		char buf_time[50];
		struct tm *time_pt=NULL;
		//create a timer for watering (Water pump ON)

		time_pt = rtc_get_time();
		sprintf(buf_time, "Water pot %d at %d:%d:%d - %d/%d", pot+1, time_pt->hour, time_pt->min, time_pt->sec, time_pt->mday, time_pt->mon);
		UB_Uart_SendString(COM3, buf_time, LFCR);
//		time_pt->min = time_pt->min + allPots[pot].water_time;
//		rtc_set_alarm(time_pt);
//		Delay(0x3FFF);
//		time_pt = rtc_get_alarm();
//		sprintf(buf_time, "Water stop Alarm set %d:%d:%d", time_pt->hour, time_pt->min, time_pt->sec);
//		UB_Uart_SendString(COM3, buf_time, LFCR);
//		Delay(0x3FFF);

		//Start water pump
		GPIO_ResetBits(allPots[pot].water_pump.PORT, allPots[pot].water_pump.PIN);
		//Energize solenoid
		GPIO_ResetBits(allPots[pot].solenoid.PORT, allPots[pot].solenoid.PIN);

		//UB_Uart_SendString(COM3,"Started watering",LFCR);

		//wait for alarm
		if (xTimerStart(timerHndl1Sec, 0)!=pdPASS) {
			UB_Uart_SendString(COM3,"Failure",LFCR);
			Delay(0xFFFF);
		}

		//UB_Uart_SendString(COM3,"Started timer",LFCR);
		vTaskSuspend(NULL);
//		while(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_12)==Bit_SET){
//			vTaskDelay(1 * configTICK_RATE_HZ );
//			UB_Uart_SendString(COM3,"wait alarm",LFCR);
//		}


//		time_pt = rtc_get_time();
//		sprintf(buf_time, "Started watering at %d:%d:%d", time_pt->hour, time_pt->min, time_pt->sec);
//		UB_Uart_SendString(COM3, buf_time, LFCR);
//		Delay(0xFFFF);
		//Wait for the timer finished
		//vTaskSuspend(waterTaskHndl);

		//Stop water pump
		GPIO_SetBits(allPots[pot].water_pump.PORT, allPots[pot].water_pump.PIN);
		//De-energize solenoid
		GPIO_SetBits(allPots[pot].solenoid.PORT, allPots[pot].solenoid.PIN);

		time_pt = rtc_get_time();
		sprintf(buf_time, "Stopped watering pot %d at %d:%d:%d", pot+1, time_pt->hour, time_pt->min, time_pt->sec);
		UB_Uart_SendString(COM3, buf_time, LFCR);

	return 0;
}

//offset is the index of ADC1Data where the first moisture value is
uint8_t CheckMoisture(uint8_t PotNum){
	uint16_t avg = 0;
	uint8_t i;
	char buf[50];
	uint16_t* ADC1Data=NULL;
	ADC1Data=getSensorValues();

	//sprintf(buf, "sensors number =  %d ", allPots[PotNum].num_sensors);
	//UB_Uart_SendString(COM3, buf, LFCR);

	//sprintf(buf, "threshold =  %d mV", allPots[PotNum].threshold);
	//UB_Uart_SendString(COM3, buf, LFCR);

	for (i=0; i<allPots[PotNum].num_sensors; i++){
		avg+=ADC1Data[(allPots[PotNum].offset)+i];

	}

	//sprintf(buf, "sum =  ", avg);
	//UB_Uart_SendString(COM3, buf, LFCR);

	avg= avg/allPots[PotNum].num_sensors;

	//sprintf(buf, "avg =  %d mV", avg);
	//UB_Uart_SendString(COM3, buf, LFCR);
	//sprintf(buf, "avg =  %d mV", avg);
	//UB_Uart_SendString(COM3, buf, LFCR);
	if(avg<allPots[PotNum].threshold){
		sprintf(buf, "Pot %d is DRY with Average %d mV", PotNum+1, avg);
		UB_Uart_SendString(COM3, buf, LFCR);
		return DRY; //need to water
	}
	sprintf(buf, "Pot %d is MOIST with Average %d mV", PotNum+1, avg);
	UB_Uart_SendString(COM3, buf, LFCR);
	//no need to water
	return MOIST;
}

/**
 * TASK 3: Receive data via Inter-Process Communication (IPC) and send it out
 *
 */

//void SendDataOut_IPC(void *pvParameters) {
//	//TODO: Receive monitored values from sensors and send them over the air to be processed elsewhere (with NRF24L?)
//	int sig;
//	portBASE_TYPE status;
//
//	while (1) {
//		status = xQueueReceive(pbq, &sig, portMAX_DELAY); /* Receive Message */
//		/* portMAX_DELAY blocks task indefinitely if queue is empty */
//		if(status == pdTRUE) {
//
//
//		}
//	}
//}

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
//void DetectButtonPress(void *pvParameters){
//	//TODO: still left over from original code. Decide if a button press is relevant for the application.
//	int sig = 1;
//	while (1) {
//		/* Detect Button Press  */
//		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)>0) {
//			while(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)>0){
//				vTaskDelay(pdMS_TO_TICKS(500)); /* Button Debounce Delay */
//			}
//			while(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)==0){
//				vTaskDelay(pdMS_TO_TICKS(500)); /* Button Debounce Delay */
//			}
//
//			xQueueSendToBack(pbq, &sig, 0); /* Send Message */
//			UB_Uart_SendString(COM3,"Button pressed",LFCR);
//		}
//	}
//}



//Timer functions

static void vTimerCallback1SecExpired(xTimerHandle pxTimer) {
	vTaskResume(waterTaskHndl);
}
