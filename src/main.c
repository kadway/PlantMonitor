//#include <stdio.h>
//#include <stdlib.h>

/* Board includes */
//#include "stm32f4_discovery.h"
//#include "stm32f4xx_conf.h"
/*
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_dma.h"*/

//#include "FreeRTOS.h"
//#include "task.h"
//#include "timers.h"
//#include "semphr.h"

/* UART library include */
//#include "stm32_ub_uart.h"

#include "platform.h"

void createTasks(void);


int main(void){

	initHW();
	//debug


	//create the necessary tasks
	createTasks();
	/* Start the RTOS Scheduler */
	vTaskStartScheduler();


	/* HALT */
	while(1);
}


