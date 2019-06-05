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
char sAdcValue[24];
uint16_t i = 0;


//Semaphore handlers
SemaphoreHandle_t xSemaphore = NULL;

bool dataReady = 0;
bool alarm_not_fired = 1;

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
