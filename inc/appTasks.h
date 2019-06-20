/* Kernel includes. */
//#include "FreeRTOS.h"
//#include "task.h"
//#include "timers.h"
//#include "semphr.h"

#include "platform.h"

#define POT1_THRESHOLD 2000 //garlic
#define POT2_THRESHOLD 2300 //tomato
#define POT3_THRESHOLD 2200 //tamarillo
#define POT4_THRESHOLD 2200 //paprika
#define POT5_THRESHOLD 0  //potatos
#define POT6_THRESHOLD 2300 //tomato

#define POT1_NUM_SENS 4
#define POT2_NUM_SENS 1
#define POT3_NUM_SENS 2
#define POT4_NUM_SENS 2
#define POT5_NUM_SENS 1
#define POT6_NUM_SENS 1

#define POT1_OFFS 0
#define POT2_OFFS 4
#define POT3_OFFS 7
#define POT4_OFFS 10
#define POT5_OFFS 5
#define POT6_OFFS 9

#define WATER_PUMP1_PORT GPIOB
#define WATER_PUMP1_PIN  GPIO_Pin_8
#define WATER_PUMP2_PORT GPIOB
#define WATER_PUMP2_PIN  GPIO_Pin_9

//defines for GPIO outputs
#define POT1_WATER_PUMP_PORT WATER_PUMP1_PORT
#define POT1_WATER_PUMP_PIN  WATER_PUMP1_PIN
#define POT2_WATER_PUMP_PORT WATER_PUMP1_PORT
#define POT2_WATER_PUMP_PIN  WATER_PUMP1_PIN
#define POT3_WATER_PUMP_PORT WATER_PUMP1_PORT
#define POT3_WATER_PUMP_PIN  WATER_PUMP1_PIN

#define POT4_WATER_PUMP_PORT WATER_PUMP2_PORT
#define POT4_WATER_PUMP_PIN  WATER_PUMP2_PIN
#define POT5_WATER_PUMP_PORT WATER_PUMP2_PORT
#define POT5_WATER_PUMP_PIN  WATER_PUMP2_PIN
#define POT6_WATER_PUMP_PORT WATER_PUMP2_PORT
#define POT6_WATER_PUMP_PIN  WATER_PUMP2_PIN

#define POT1_SOLENOID_PORT GPIOA
#define POT1_SOLENOID_PIN  GPIO_Pin_8
#define POT2_SOLENOID_PORT GPIOA
#define POT2_SOLENOID_PIN  GPIO_Pin_9
#define POT3_SOLENOID_PORT GPIOA
#define POT3_SOLENOID_PIN  GPIO_Pin_10
#define POT4_SOLENOID_PORT GPIOD
#define POT4_SOLENOID_PIN  GPIO_Pin_11
#define POT5_SOLENOID_PORT GPIOD
#define POT5_SOLENOID_PIN  GPIO_Pin_12
#define POT6_SOLENOID_PORT GPIOD
#define POT6_SOLENOID_PIN  GPIO_Pin_13

/* PE 0,1,2,3,4,5,6
 * PD 0,1,2,3,4,5,6,7
 * PC 12 */

#define GPIOE_SENSORS_SUPPLY_PINS (GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 )
#define GPIOD_SENSORS_SUPPLY_PINS (GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 )

#define POT1_SENSOR1_SUPPLY_PORT GPIOE
#define POT1_SENSOR1_SUPPLY_PIN  GPIO_Pin_0
#define POT1_SENSOR2_SUPPLY_PORT GPIOE
#define POT1_SENSOR2_SUPPLY_PIN  GPIO_Pin_1
#define POT1_SENSOR3_SUPPLY_PORT GPIOE
#define POT1_SENSOR3_SUPPLY_PIN  GPIO_Pin_2
#define POT1_SENSOR4_SUPPLY_PORT GPIOE
#define POT1_SENSOR4_SUPPLY_PIN  GPIO_Pin_3

//Only 1 moisture sensor for POT2
#define POT2_SENSOR1_SUPPLY_PORT GPIOE
#define POT2_SENSOR1_SUPPLY_PIN  GPIO_Pin_4
#define POT2_SENSOR2_SUPPLY_PORT GPIOE
#define POT2_SENSOR2_SUPPLY_PIN  GPIO_Pin_4
#define POT2_SENSOR3_SUPPLY_PORT GPIOE
#define POT2_SENSOR3_SUPPLY_PIN  GPIO_Pin_4
#define POT2_SENSOR4_SUPPLY_PORT GPIOE
#define POT2_SENSOR4_SUPPLY_PIN  GPIO_Pin_4

//Only 2 moisture sensors for POT3
#define POT3_SENSOR1_SUPPLY_PORT GPIOE
#define POT3_SENSOR1_SUPPLY_PIN  GPIO_Pin_5
#define POT3_SENSOR2_SUPPLY_PORT GPIOE
#define POT3_SENSOR2_SUPPLY_PIN  GPIO_Pin_6
#define POT3_SENSOR3_SUPPLY_PORT GPIOE
#define POT3_SENSOR3_SUPPLY_PIN  GPIO_Pin_5
#define POT3_SENSOR4_SUPPLY_PORT GPIOE
#define POT3_SENSOR4_SUPPLY_PIN  GPIO_Pin_6

#define POT4_SENSOR1_SUPPLY_PORT GPIOD
#define POT4_SENSOR1_SUPPLY_PIN  GPIO_Pin_0
#define POT4_SENSOR2_SUPPLY_PORT GPIOD
#define POT4_SENSOR2_SUPPLY_PIN  GPIO_Pin_1
#define POT4_SENSOR3_SUPPLY_PORT GPIOD
#define POT4_SENSOR3_SUPPLY_PIN  GPIO_Pin_2
#define POT4_SENSOR4_SUPPLY_PORT GPIOD
#define POT4_SENSOR4_SUPPLY_PIN  GPIO_Pin_3

//Only 2 moisture sensors for POT5
#define POT5_SENSOR1_SUPPLY_PORT GPIOD
#define POT5_SENSOR1_SUPPLY_PIN  GPIO_Pin_4
#define POT5_SENSOR2_SUPPLY_PORT GPIOD
#define POT5_SENSOR2_SUPPLY_PIN  GPIO_Pin_5
#define POT5_SENSOR3_SUPPLY_PORT GPIOD
#define POT5_SENSOR3_SUPPLY_PIN  GPIO_Pin_4
#define POT5_SENSOR4_SUPPLY_PORT GPIOD
#define POT5_SENSOR4_SUPPLY_PIN  GPIO_Pin_5

//Only 1 moisture sensor for POT6
#define POT6_SENSOR1_SUPPLY_PORT GPIOD
#define POT6_SENSOR1_SUPPLY_PIN  GPIO_Pin_6
#define POT6_SENSOR2_SUPPLY_PORT GPIOD
#define POT6_SENSOR2_SUPPLY_PIN  GPIO_Pin_6
#define POT6_SENSOR3_SUPPLY_PORT GPIOD
#define POT6_SENSOR3_SUPPLY_PIN  GPIO_Pin_6
#define POT6_SENSOR4_SUPPLY_PORT GPIOD
#define POT6_SENSOR4_SUPPLY_PIN  GPIO_Pin_6

#define MAX_SENS 4
//#define DEFAULT_WATER_TIME 3000
#define DEFAULT_WATER_TIME 10000 //~31 seconds

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
	uint8_t offset;
	bool status;
	uint16_t water_time;
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

uint8_t CheckMoisture(uint8_t PotNum);
uint8_t WaterPot(uint8_t pot, int8_t temperature);

extern struct tm time;

static void vTimerCallback1SecExpired(xTimerHandle pxTimer);
