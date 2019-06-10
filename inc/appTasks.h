/* Kernel includes. */
//#include "FreeRTOS.h"
//#include "task.h"
//#include "timers.h"
//#include "semphr.h"

#include "platform.h"


/*Task handles*/
TaskHandle_t sensorTaskHndl = NULL;
TaskHandle_t waterTaskHndl = NULL;
TaskHandle_t sleepTaskHndl = NULL;

xQueueHandle pbq;

/*Global Variables*/
char sAdcValue[50];
uint16_t i = 0;


//Semaphore handlers
SemaphoreHandle_t xSemaphore = NULL;

bool adcready = 0;

char ADC1_Pins [14][4] = { "PA1\0", "PA2\0", "PA3\0", "PA4\0","PA5\0", "PA6\0", "PA7\0", "PB0\0", "PB1\0","PC1\0", "PC2\0", "PC3\0", "PC4\0", "PC5\0"};

void createTasks(void);
void PoolSensors(void *pvParameters);
void WaterPlants(void *pvParameters);
void SendDataOut_IPC(void *pvParameters);
void SleepAlarm(void *pvParameters);
void DetectButtonPress(void *pvParameters);

xTimerHandle timerHndl1Sec;
uint16_t counter = 0;


// statically allocated

extern struct tm time;

static void vTimerCallback1SecExpired(xTimerHandle pxTimer);
