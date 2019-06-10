/* Kernel includes. */
//#include "FreeRTOS.h"
//#include "task.h"
//#include "timers.h"
//#include "semphr.h"

#include "platform.h"

#define POT1_THRESHOLD 900
#define POT2_THRESHOLD 900
#define POT3_THRESHOLD 900
#define POT4_THRESHOLD 900
#define POT5_THRESHOLD 900
#define POT6_THRESHOLD 900

#define POT1_NUM_SENS 4
#define POT2_NUM_SENS 1
#define POT3_NUM_SENS 2
#define POT4_NUM_SENS 4
#define POT5_NUM_SENS 2
#define POT6_NUM_SENS 1

#define POT1_OFFS 0
#define POT2_OFFS 4
#define POT3_OFFS 5
#define POT4_OFFS 7
#define POT5_OFFS 11
#define POT6_OFFS 13

//defines for GPIO outputs
#define POT1_WATER_PUMP_PORT GPIOA
#define POT1_WATER_PUMP_PIN  GPIO_Pin_2
#define POT2_WATER_PUMP_PORT GPIOA
#define POT2_WATER_PUMP_PIN  GPIO_Pin_2
#define POT3_WATER_PUMP_PORT GPIOA
#define POT3_WATER_PUMP_PIN  GPIO_Pin_2
#define POT4_WATER_PUMP_PORT GPIOA
#define POT4_WATER_PUMP_PIN  GPIO_Pin_2
#define POT5_WATER_PUMP_PORT GPIOA
#define POT5_WATER_PUMP_PIN  GPIO_Pin_2
#define POT6_WATER_PUMP_PORT GPIOA
#define POT6_WATER_PUMP_PIN  GPIO_Pin_2

#define POT1_SOLENOID_PORT GPIOA
#define POT1_SOLENOID_PIN  GPIO_Pin_2
#define POT2_SOLENOID_PORT GPIOA
#define POT2_SOLENOID_PIN  GPIO_Pin_2
#define POT3_SOLENOID_PORT GPIOA
#define POT3_SOLENOID_PIN  GPIO_Pin_2
#define POT4_SOLENOID_PORT GPIOA
#define POT4_SOLENOID_PIN  GPIO_Pin_2
#define POT5_SOLENOID_PORT GPIOA
#define POT5_SOLENOID_PIN  GPIO_Pin_2
#define POT6_SOLENOID_PORT GPIOA
#define POT6_SOLENOID_PIN  GPIO_Pin_2

#define POT1_SENSOR1_SUPPLY_PORT GPIOA
#define POT1_SENSOR1_SUPPLY_PIN  GPIO_Pin_2
#define POT1_SENSOR2_SUPPLY_PORT GPIOA
#define POT1_SENSOR2_SUPPLY_PIN  GPIO_Pin_2
#define POT1_SENSOR3_SUPPLY_PORT GPIOA
#define POT1_SENSOR3_SUPPLY_PIN  GPIO_Pin_2
#define POT1_SENSOR4_SUPPLY_PORT GPIOA
#define POT1_SENSOR4_SUPPLY_PIN  GPIO_Pin_2

#define POT2_SENSOR1_SUPPLY_PORT GPIOA
#define POT2_SENSOR1_SUPPLY_PIN  GPIO_Pin_2
#define POT2_SENSOR2_SUPPLY_PORT GPIOA
#define POT2_SENSOR2_SUPPLY_PIN  GPIO_Pin_2
#define POT2_SENSOR3_SUPPLY_PORT GPIOA
#define POT2_SENSOR3_SUPPLY_PIN  GPIO_Pin_2
#define POT2_SENSOR4_SUPPLY_PORT GPIOA
#define POT2_SENSOR4_SUPPLY_PIN  GPIO_Pin_2

#define POT3_SENSOR1_SUPPLY_PORT GPIOA
#define POT3_SENSOR1_SUPPLY_PIN  GPIO_Pin_2
#define POT3_SENSOR2_SUPPLY_PORT GPIOA
#define POT3_SENSOR2_SUPPLY_PIN  GPIO_Pin_2
#define POT3_SENSOR3_SUPPLY_PORT GPIOA
#define POT3_SENSOR3_SUPPLY_PIN  GPIO_Pin_2
#define POT3_SENSOR4_SUPPLY_PORT GPIOA
#define POT3_SENSOR4_SUPPLY_PIN  GPIO_Pin_2

#define POT4_SENSOR1_SUPPLY_PORT GPIOA
#define POT4_SENSOR1_SUPPLY_PIN  GPIO_Pin_2
#define POT4_SENSOR2_SUPPLY_PORT GPIOA
#define POT4_SENSOR2_SUPPLY_PIN  GPIO_Pin_2
#define POT4_SENSOR3_SUPPLY_PORT GPIOA
#define POT4_SENSOR3_SUPPLY_PIN  GPIO_Pin_2
#define POT4_SENSOR4_SUPPLY_PORT GPIOA
#define POT4_SENSOR4_SUPPLY_PIN  GPIO_Pin_2

#define POT5_SENSOR1_SUPPLY_PORT GPIOA
#define POT5_SENSOR1_SUPPLY_PIN  GPIO_Pin_2
#define POT5_SENSOR2_SUPPLY_PORT GPIOA
#define POT5_SENSOR2_SUPPLY_PIN  GPIO_Pin_2
#define POT5_SENSOR3_SUPPLY_PORT GPIOA
#define POT5_SENSOR3_SUPPLY_PIN  GPIO_Pin_2
#define POT5_SENSOR4_SUPPLY_PORT GPIOA
#define POT5_SENSOR4_SUPPLY_PIN  GPIO_Pin_2

#define POT6_SENSOR1_SUPPLY_PORT GPIOA
#define POT6_SENSOR1_SUPPLY_PIN  GPIO_Pin_2
#define POT6_SENSOR2_SUPPLY_PORT GPIOA
#define POT6_SENSOR2_SUPPLY_PIN  GPIO_Pin_2
#define POT6_SENSOR3_SUPPLY_PORT GPIOA
#define POT6_SENSOR3_SUPPLY_PIN  GPIO_Pin_2
#define POT6_SENSOR4_SUPPLY_PORT GPIOA
#define POT6_SENSOR4_SUPPLY_PIN  GPIO_Pin_2

#define MAX_SENS 4

typedef enum{
	POT1 = POT1_OFFS,
	POT2 = POT2_OFFS,
	POT3 = POT3_OFFS,
	POT4 = POT4_OFFS,
	POT5 = POT5_OFFS,
	POT6 = POT6_OFFS
}POT_offset_t;

typedef struct{
	GPIO_TypeDef* PORT;     // Port
	const uint16_t PIN;     // Pin
}Gpio_outputs_t;


typedef struct {
	POT_offset_t offset;
	uint16_t threshold;
	uint8_t num_sensors;
	Gpio_outputs_t water_pump;
	Gpio_outputs_t solenoid;
	Gpio_outputs_t sens_sup[MAX_SENS];
}Pot_t;

#define NUM_POTS 6
#define DRY 1
#define MOIST 0

/*Task handles*/
TaskHandle_t sensorTaskHndl = NULL;
TaskHandle_t waterTaskHndl = NULL;
TaskHandle_t sleepTaskHndl = NULL;

xQueueHandle pbq;

/*Global Variables*/
char sAdcValue[50];

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

uint8_t CheckMoisture(uint8_t num_sensors, uint16_t threshold, uint8_t offset, uint16_t* data);

extern struct tm time;

static void vTimerCallback1SecExpired(xTimerHandle pxTimer);
