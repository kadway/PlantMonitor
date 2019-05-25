/*
 * DS RTC Library: DS1307 and DS3231 driver library
 * (C) 2011 Akafugu Corporation
 * (C) 2019 Joao Goncalves
 * This program is free software; you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 2 of the License, or (at your option) any later
 * version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
 * PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 */

#ifndef DS1307_H
#define DS1307_H

#include <stdbool.h>

#include "hal_i2c.h"
#include "platform.h"

/** Time structure
 * 
 * Both 24-hour and 12-hour time is stored, and is always updated when rtc_get_time is called.
 * 
 * When setting time and alarm, 24-hour mode is always used.
 *
 * If you run your clock in 12-hour mode:
 * - set time hour to store in twelveHour and set am to true or false.
 * - call rtc_12h_translate (this will put the correct value in hour, so you don't have to
 *   calculate it yourself.
 * - call rtc_set_alarm or rtc_set_clock
 *
 * Note that rtc_set_clock_s, rtc_set_alarm_s, rtc_get_time_s, rtc_set_alarm_s always operate in 24-hour mode
 * and translation has to be done manually (you can call rtc_24h_to_12h to perform the calculation)
 *
 */
struct tm {
	uint8_t sec;      // 0 to 59
	uint8_t min;      // 0 to 59
	uint8_t hour;     // 0 to 23
	uint8_t mday;     // 1 to 31
	uint8_t mon;      // 1 to 12
	uint8_t year;     // year-99
	uint8_t wday;     // 1-7

    // 12-hour clock data
    bool am; // true for AM, false for PM
    int twelveHour; // 12 hour clock time
};

//typedef struct ds3231_control_s{
//	// 0x0e: control register
//	 uint8_t  EOSCN:1; // When set to logic 0, the oscillator is started. When set to logic 1, the oscillator is stopped when the DS3231 switches to V BAT .
//	 uint8_t  BBSQW:1; // When set to logic 1 with INTCN = 0 and V CC < V PF , this bit enables the square wave. When BBSQW is logic 0, the INT/SQW pin goes high impedance when V CC < V PF . This bit is disabled (logic 0) when power is first applied.
//	 uint8_t  CONV:1; // Setting this bit to 1 forces the temperature sensor to convert the temperature into digital code and execute the TCXO algorithm to update the capacitance array to the oscillator.
//	 uint8_t  RS2:1; // 0       | 0            | 1			   | 1
//	 uint8_t  RS1:1; // 0 = 1Hz | 1 = 1.024kHz | 0 = 4.096kHz  | 1 = 8.192kHz
//	 uint8_t  INTCN:1; // This bit controls the INT/SQW signal. When the INTCN bit is set to logic 0, a square wave is output on the INT/SQW pin. When bit is 0, the INT/SQW pin goes low when an alarm flag is set.
//	 uint8_t  A2IE:1; // Alarm 2 Interrupt Enable
//	 uint8_t  A1IE:1; // Alarm 1 Interrupt Enable
//}ds3231_control ;
//
//typedef struct ds3231_status_s{
// //0x0f: status
//  uint8_t  OSF:1; // A logic 1 in this bit indicates that the oscillator either is stopped or was stopped. This bit remains at logic 1 until written to logic 0
//  uint8_t  RES0:1;   //reserved
//  uint8_t  RES1:1;   //reserved
//  uint8_t  RES2:1;   //reserved
//  uint8_t  EN32kHz:1;//Enable Square ware = 1 / Enable Interrupt = 0
//  uint8_t  BSY:1;    //The device is busy executing TCXO functions
//  uint8_t  A2F:1;    //alarm 2 flag
//  uint8_t  A1F:1;    //alarm 1 flag
//}ds3231_status;


typedef struct ds3231_control_s{
	// 0x0e: control register
	uint8_t  A1IE:1; // Alarm 1 Interrupt Enable
	uint8_t  A2IE:1; // Alarm 2 Interrupt Enable
	uint8_t  INTCN:1; // This bit controls the INT/SQW signal. When the INTCN bit is set to logic 0, a square wave is output on the INT/SQW pin. When bit is 0, the INT/SQW pin goes low when an alarm flag is set.
	uint8_t  RS1:1; // 0 = 1Hz | 1 = 1.024kHz | 0 = 4.096kHz  | 1 = 8.192kHz
	uint8_t  RS2:1; // 0       | 0            | 1			   | 1
	uint8_t  CONV:1; // Setting this bit to 1 forces the temperature sensor to convert the temperature into digital code and execute the TCXO algorithm to update the capacitance array to the oscillator.
	uint8_t  BBSQW:1; // When set to logic 1 with INTCN = 0 and V CC < V PF , this bit enables the square wave. When BBSQW is logic 0, the INT/SQW pin goes high impedance when V CC < V PF . This bit is disabled (logic 0) when power is first applied
	uint8_t  EOSCN:1; // When set to logic 0, the oscillator is started. When set to logic 1, the oscillator is stopped when the DS3231 switches to V BAT .
}ds3231_control ;

typedef struct ds3231_status_s{
 //0x0f: status
  uint8_t  A1F:1;    //alarm 1 flag
  uint8_t  A2F:1;    //alarm 2 flag
  uint8_t  BSY:1;    //The device is busy executing TCXO functions
  uint8_t  EN32kHz:1;//Enable Square ware = 1 / Enable Interrupt = 0
  uint8_t  RES2:1;   //reserved
  uint8_t  RES1:1;   //reserved
  uint8_t  RES0:1;   //reserved
  uint8_t  OSF:1; // A logic 1 in this bit indicates that the oscillator either is stopped or was stopped. This bit remains at logic 1 until written to logic 0
}ds3231_status;

/*
* DS3231 register map
*/
//  00h-06h: seconds, minutes, hours, day-of-week, date, month, year (all in BCD)
//       bit 7 should be set to zero: The DS3231 clock is always running
#define  A1M1_ADDR 0x07         // Alarm 1 seconds
#define  A1M2_ADDR 0x08         // Alarm 1 minutes
#define  A1M3_ADDR 0x09         // Alarm 1 hour (bit6 is am/pm flag in 12h mode)
#define  A1M4_ADDR 0x0a         // Alarm 1 day/date (bit6: 1 for day, 0 for date)
#define  A2M2_ADDR 0x0b         // A2M2  Alarm 2 minutes
#define  A2M3_ADDR 0x0c         // Alarm 2 hour (bit6 is am/pm flag in 12h mode)
#define  A2M4_ADDR 0x0d         // Alarm 2 day/data (bit6: 1 for day, 0 for date)
#define  CONTROL_ADDR 0x0e      //Control register
#define  STATUS_ADDR 0x0f       //Control/status register
#define  AGING_OFFSET_ADDR 0x10 // aging offset (signed)
#define  TEMP_MSB_ADDR 0x11     //MSB of temp (signed)
#define  TEMP_LSB_ADDR 0x12     //LSB of temp in bits 7 and 6 (0.25 degrees for each 00, 01, 10, 11)

// statically allocated 
extern struct tm time;

// Initialize the RTC and autodetect type (DS1307 or DS3231)
void rtc_init(void);
bool rtc_check_status(uint8_t* status);

void rtc_write_byte(uint8_t* b, uint8_t offset);
void rtc_read_byte(uint8_t* status, uint8_t offset);

// Get/set time
// Gets the time: Supports both 24-hour and 12-hour mode
struct tm* rtc_get_time(void);
// Gets the time: 24-hour mode only
void rtc_get_time_s(uint8_t* hour, uint8_t* min, uint8_t* sec);
// Sets the time: Supports both 24-hour and 12-hour mode
void rtc_set_time(struct tm* tm_);
// Sets the time: Supports 12-hour mode only
void rtc_set_time_s(uint8_t hour, uint8_t min, uint8_t sec);

// Read Temperature (DS3231 only)
void  ds3231_get_temp_int(int8_t* i, uint8_t* f);
void rtc_force_temp_conversion(void);
void readTemperatureRTC(int8_t* integer, uint8_t* fractional);

  // Auxillary functions
enum RTC_SQW_FREQ { FREQ_1 = 0, FREQ_1024, FREQ_4096, FREQ_8192 };

void rtc_SQW_enable(bool enable);
void rtc_SQW_set_freq(enum RTC_SQW_FREQ freq);
void rtc_osc32kHz_enable(bool enable);

// Alarm functionality
void rtc_reset_alarm(void);
void rtc_set_alarm(struct tm* tm_);
void rtc_set_alarm_s(uint8_t hour, uint8_t min, uint8_t sec);
struct tm* rtc_get_alarm(void);
void rtc_get_alarm_s(uint8_t* hour, uint8_t* min, uint8_t* sec);
bool rtc_check_alarm(void);  

#endif
