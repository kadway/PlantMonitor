/*
 * DS RTC Library: DS1307 and DS3231 driver library
 * (C) 2011 Akafugu Corporation
 * (C) 2019 Joao Goncalves
 *
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

/*
 *
 * DS3231 register map
 *
 *  00h-06h: seconds, minutes, hours, day-of-week, date, month, year (all in BCD)
 *       bit 7 should be set to zero: The DS3231 clock is always running
 *  07h: A1M1  Alarm 1 seconds
 *  08h: A1M2  Alarm 1 minutes
 *  09h: A1M3  Alarm 1 hour (bit6 is am/pm flag in 12h mode)
 *  0ah: A1M4  Alarm 1 day/date (bit6: 1 for day, 0 for date)
 *  0bh: A2M2  Alarm 2 minutes
 *  0ch: A2M3  Alarm 2 hour (bit6 is am/pm flag in 12h mode)
 *  0dh: A2M4  Alarm 2 day/data (bit6: 1 for day, 0 for date)
 *       <see data sheet page12 for Alarm register mask bit tables:
 *        for alarm when hours, minutes and seconds match set 1000 for alarm 1>
 *  0eh: control
 *      bit7: !EOSC
 *      bit6: BBSQW
 *      bit5: CONV
 *      bit4: RS2
 *      bit3: RS1
 *      bit2: INTCN
 *      bit1: A2IE
 *      bit0: A1IE
 *  0fh: control/status
 *      bit7: OSF
 *      bit6: 0
 *      bit5: 0
 *      bit4: 0
 *      bit3: EN32kHz
 *      bit2: BSY
 *      bit1: A2F alarm 2 flag
 *      bit0: A1F alarm 1 flag
 * 10h: aging offset (signed)
 * 11h: MSB of temp (signed)
 * 12h: LSB of temp in bits 7 and 6 (0.25 degrees for each 00, 01, 10, 11)
 *
 */


#define TRUE 1
#define FALSE 0

#include "rtc.h"


//#define RTC_ADDR 0x68 // I2C address not shifted
#define RTC_ADDR 0xD0 // I2C address shifted right
#define CH_BIT 7 // clock halt bit

// statically allocated structure for time value
struct tm time;

//for debug
char buf[20];

uint8_t dec2bcd(uint8_t d)
{
  return ((d/10 * 16) + (d % 10));
}

uint8_t bcd2dec(uint8_t b)
{
  return ((b/16 * 10) + (b % 16));
}

void rtc_read_byte(uint8_t* status, uint8_t offset)
{
	I2C_start(I2C1, RTC_ADDR, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
	I2C_write(I2C1, offset);
	I2C_stop(I2C1);
	Delay(0x3FFF);
	I2C_start(I2C1, RTC_ADDR, I2C_Direction_Receiver); // start a transmission in Master receiver mode
	*status = I2C_read_nack(I2C1); // read one byte and don't request another byte, stop transmission
}

void rtc_write_byte(uint8_t* b, uint8_t offset)
{
//	char buf[25];
//	sprintf(buf, "write byte: %#04x", *b);
//	UB_Uart_SendString(COM2, buf, LFCR);
	I2C_start(I2C1, RTC_ADDR, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
	I2C_write(I2C1, offset);
	Delay(0x3FFF);
	I2C_write(I2C1, *b);
	I2C_stop(I2C1);
}

bool rtc_check_status(uint8_t* status){

	if (*status==0x00){
		return 0;
	}
	else{
		return 1;
	}
}

void rtc_init(void)
{
	//bit fields for control and status registers
	ds3231_control control;
	ds3231_status status;

	//UB_Uart_SendString(COM2, "RTC INIT.", LFCR);
	do{
		control.INTCN=0;
		control.A1IE=0;
		control.A2IE=0;
		control.BBSQW=0;
		control.EOSCN=0;

		status.A1F=0;
		status.A2F=0;
		status.EN32kHz=0;
		status.OSF=0;

		rtc_write_byte((uint8_t*)&control, CONTROL_ADDR);
		Delay(0x3FFF);
		rtc_write_byte((uint8_t*)&status, STATUS_ADDR);
		Delay(0x3FFF);
		//status 0x00 is good (Oscillator is running, device not busy and no alarm flag is set)
	}while(rtc_check_status((uint8_t*) &status));
	Delay(0x3FFF);

	control.INTCN=1;
	rtc_write_byte((uint8_t*)&control, CONTROL_ADDR);
	Delay(0x3FFF);

	rtc_read_byte((uint8_t*) &status, STATUS_ADDR);
	sprintf(buf, "Status: %#04x", status);
	UB_Uart_SendString(COM2, buf, LFCR);
//
//	control.INTCN=1;
//	control.A1IE=0;
//	control.A2IE=0;
//	control.BBSQW=0;
//	control.EOSCN=0;
//
//	rtc_write_byte((uint8_t*)&control, CONTROL_ADDR);
//
//	rtc_read_byte((uint8_t*) &control, CONTROL_ADDR);
//	sprintf(buf, "Control: %#04x", status);
//	UB_Uart_SendString(COM2, buf, LFCR);

#ifdef SETTIME
	time.sec=0;
	time.min=11;
	time.hour=12;
	time.mday=25;
	time.mon=5;
	time.year=19;
	time.wday=2;
	rtc_set_time(&time);
#endif

}


struct tm* rtc_get_time(void)
{
	uint8_t rtc[9];
	uint8_t century = 0;

	// read 7 bytes starting from register 0
	// sec, min, hour, day-of-week, date, month, year

	I2C_start(I2C1, RTC_ADDR, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
	I2C_write(I2C1, 0x0);
	I2C_stop(I2C1);
	Delay(0x3FFF);
	I2C_start(I2C1, RTC_ADDR, I2C_Direction_Receiver); // start a transmission in Master receiver mode

	for (uint8_t i = 0; i < 6; i++) {
		rtc[i] = I2C_read_ack(I2C1);// read one byte and request another byte
	}
	rtc[7] = I2C_read_nack(I2C1); // read one byte and don't request another byte, stop transmission


	// Clear clock halt bit from read data
	// This starts the clock for a DS1307, and has no effect for a DS3231
	//rtc[0] &= ~(_BV(CH_BIT)); // clear bit

	time.sec = bcd2dec(rtc[0]);
	time.min = bcd2dec(rtc[1]);
	time.hour = bcd2dec(rtc[2]);
	time.mday = bcd2dec(rtc[4]);
	time.mon = bcd2dec(rtc[5] & 0x1F); // returns 1-12
	//century = (rtc[5] & 0x80) >> 7;
	//time.year = century == 1 ? 2000 + bcd2dec(rtc[6]) : 1900 + bcd2dec(rtc[6]); // year 0-99
	time.year = bcd2dec(rtc[6]); // year 0-99
	time.wday = bcd2dec(rtc[3]); // returns 1-7

	if (time.hour == 0) {
		time.twelveHour = 0;
		time.am = 1;
	} else if (time.hour < 12) {
		time.twelveHour = time.hour;
		time.am = 1;
	} else {
		time.twelveHour = time.hour - 12;
		time.am = 0;
	}

	return &time;
}

void rtc_get_time_s(uint8_t* hour, uint8_t* min, uint8_t* sec)
{
	uint8_t rtc[9];

	// read 7 bytes starting from register 0
	// sec, min, hour, day-of-week, date, month, year
	I2C_start(I2C1, RTC_ADDR, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
	I2C_write(I2C1, 0x0);
	I2C_stop(I2C1);

	I2C_start(I2C1, RTC_ADDR, I2C_Direction_Receiver); // start a transmission in Master receiver mode

	for (uint8_t i = 0; i < 6; i++) {
		rtc[i] = I2C_read_ack(I2C1);// read one byte and request another byte
	}
	rtc[7] = I2C_read_nack(I2C1); // read one byte and don't request another byte, stop transmission

	if (sec)  *sec =  bcd2dec(rtc[0]);
	if (min)  *min =  bcd2dec(rtc[1]);
	if (hour) *hour = bcd2dec(rtc[2]);
}

// fixme: support 12-hour mode for setting time
void rtc_set_time(struct tm* tm_)
{
	I2C_start(I2C1, RTC_ADDR, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
	I2C_write(I2C1, 0x0);
	Delay(0xFFFF);
	uint8_t century;
	if (tm_->year > 2000) {
		century = 0x80;
		tm_->year = tm_->year - 2000;
	} else {
		century = 0;
		tm_->year = tm_->year - 1900;
	}

	// clock halt bit is 7th bit of seconds: this is always cleared to start the clock
	I2C_write(I2C1, dec2bcd(tm_->sec)); // seconds
	Delay(0x0FFF);
	I2C_write(I2C1, dec2bcd(tm_->min)); // minutes
	Delay(0x0FFF);
	I2C_write(I2C1, dec2bcd(tm_->hour)); // hours
	Delay(0x0FFF);
	I2C_write(I2C1, dec2bcd(tm_->wday)); // day of week
	Delay(0x0FFF);
	I2C_write(I2C1, dec2bcd(tm_->mday)); // day
	Delay(0x0FFF);
	I2C_write(I2C1, dec2bcd(tm_->mon) + century); // month
	Delay(0x0FFF);
	I2C_write(I2C1, dec2bcd(tm_->year)); // year
	I2C_stop(I2C1);
}

void rtc_set_time_s(uint8_t hour, uint8_t min, uint8_t sec)
{
	I2C_start(I2C1, RTC_ADDR, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
	I2C_write(I2C1, 0x0);

	// clock halt bit is 7th bit of seconds: this is always cleared to start the clock
	I2C_write(I2C1, dec2bcd(sec)); // seconds
	I2C_write(I2C1, dec2bcd(min)); // minutes
	I2C_write(I2C1, dec2bcd(hour)); // hours
	
	I2C_stop(I2C1);
}

void ds3231_get_temp_int(int8_t* i, uint8_t* f)
{
	//UB_Uart_SendString(COM2, "Get temperature", LFCR);
	uint8_t msb, lsb;
	
	*i = 0;
	*f = 0;

	I2C_start(I2C1, RTC_ADDR, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
	// temp registers 0x11 and 0x12
	I2C_write(I2C1, 0x11);
	I2C_stop(I2C1);

	I2C_start(I2C1, RTC_ADDR, I2C_Direction_Receiver); // start a transmission in Master receiver mode

	// integer part (in twos complement)
	msb = I2C_read_ack(I2C1);// read one byte and request another byte
	// fraction part
	lsb = I2C_read_nack(I2C1); // read one byte and don't request another byte, stop transmission

	// integer part in entire byte
	*i = msb;
	// fractional part in top two bits (increments of 0.25)
	*f = (lsb >> 6) * 25;

	// float value can be read like so:
	 //float temp = ((((short)msb << 8) | (short)lsb) >> 6) / 4.0f;

}

void readTemperatureRTC(int8_t *integer, uint8_t *fractional){
	//UB_Uart_SendString(COM2, "Read temperature RTC", LFCR);
	//rtc_force_temp_conversion();
	ds3231_get_temp_int(integer, fractional);
};

void rtc_force_temp_conversion(void){
	UB_Uart_SendString(COM2, "Force temperature conversion", LFCR);
	// read control register (0x0E)
	I2C_start(I2C1, RTC_ADDR, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
	I2C_write(I2C1, 0x0E);
	I2C_stop(I2C1);

	I2C_start(I2C1, RTC_ADDR, I2C_Direction_Receiver); // start a transmission in Master receiver mode
	uint8_t ctrl = I2C_read_nack(I2C1); // read one byte and don't request another byte, stop transmission

	ctrl |= 0b00100000; // Set CONV bit

	// write new control register value
	I2C_start(I2C1, RTC_ADDR, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
	I2C_write(I2C1, 0x0E);
	Delay(0x0FFF);
	I2C_write(I2C1, ctrl);
	I2C_stop(I2C1);
	
	// Temp conversion is ready when control register becomes 0
	do {
		// Block until CONV is 0
		// write new control register value
		I2C_start(I2C1, RTC_ADDR, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
		I2C_write(I2C1, 0x0E);
		I2C_stop(I2C1);

		I2C_start(I2C1, RTC_ADDR, I2C_Direction_Receiver); // start a transmission in Master receiver mode

	} while ((I2C_read_nack(I2C1) & 0b00100000) != 0);
}

// Alarm functionality

void rtc_reset_alarm(void)
{
	// writing 0 to bit 7 of all four alarm 1 registers disables alarm
	uint8_t reset_val = 0;
	//UB_Uart_SendString(COM2, "reset second", LFCR);
	Delay(0x0FFF);
	rtc_write_byte(&reset_val, A1M1_ADDR); // second
	Delay(0x0FFF);
	//UB_Uart_SendString(COM2, "reset min", LFCR);
	rtc_write_byte(&reset_val, A1M2_ADDR); // minute
	Delay(0x0FFF);
	//UB_Uart_SendString(COM2, "reset hour", LFCR);
	rtc_write_byte(&reset_val, A1M3_ADDR); // hour
	//UB_Uart_SendString(COM2, "reset sec", LFCR);
	Delay(0x0FFF);
	rtc_write_byte(&reset_val, A1M4_ADDR); // day
}


void rtc_set_alarm_s(uint8_t hour, uint8_t min, uint8_t sec)
{
	if (hour > 23) return;
	if (min > 59) return;
	if (sec > 59) return;

	sec = dec2bcd(sec);
	min = dec2bcd(min);
	hour = dec2bcd(hour);

	uint8_t day = 0b10000001; // day (upper bit must be set)
	//UB_Uart_SendString(COM2, "set sec", LFCR);
	Delay(0x0FFF);
	rtc_write_byte(&sec,  A1M1_ADDR); // second
	Delay(0x0FFF);
	//UB_Uart_SendString(COM2, "set min", LFCR);
	rtc_write_byte(&min,  A1M2_ADDR); // minute
	Delay(0x0FFF);
	//UB_Uart_SendString(COM2, "set h", LFCR);
	rtc_write_byte(&hour, A1M3_ADDR); // hour
	Delay(0x0FFF);
	//UB_Uart_SendString(COM2, "set day", LFCR);
	rtc_write_byte(&day,  A1M4_ADDR); // day
	Delay(0x0FFF);
	ds3231_status status;
	rtc_read_byte((uint8_t*) &status, STATUS_ADDR);
	status.A1F = 0;// clear alarm 1 flag
	status.A2F = 0;// clear alarm 2 flag
	status.OSF = 0;// clear any OSC fault just in case
	status.EN32kHz = 0;// disable square wave if not done before
	//UB_Uart_SendString(COM2, "clear alarm", LFCR);
	Delay(0x0FFF);
	rtc_write_byte((uint8_t*) &status, STATUS_ADDR);
	Delay(0x0FFF);
	ds3231_control control;
	rtc_read_byte((uint8_t*) &control, CONTROL_ADDR);
	control.INTCN=1;//enable the INTERRUPT
	control.A1IE=1; //enable alarm1
	control.A2IE=0; //disable alarm2
	//UB_Uart_SendString(COM2, "enable interrupt", LFCR);
	Delay(0x0FFF);
	rtc_write_byte((uint8_t*) &control, CONTROL_ADDR);
}


void rtc_set_alarm(struct tm* tm_)
{
	if (!tm_) return;
	rtc_set_alarm_s(tm_->hour, tm_->min, tm_->sec);
}

void rtc_get_alarm_s(uint8_t* hour, uint8_t* min, uint8_t* sec)
{
	rtc_read_byte(sec, A1M1_ADDR);
	*sec  = bcd2dec(*sec & ~0b10000000);
	Delay(0x0FFF);
	rtc_read_byte(min, A1M2_ADDR);
	*min  = bcd2dec(*min & ~0b10000000);
	Delay(0x0FFF);
	rtc_read_byte(hour, A1M3_ADDR);
	*hour = bcd2dec(*hour & ~0b10000000);
}

struct tm* rtc_get_alarm(void)
{
	uint8_t hour, min, sec;

	rtc_get_alarm_s(&hour, &min, &sec);
	time.hour = hour;
	time.min = min;
	time.sec = sec;
	return &time;
}

bool rtc_check_alarm(void)
{

	// Alarm 1 flag (A1F) in bit 0
	uint8_t alarm1;

	rtc_read_byte(&alarm1, STATUS_ADDR);

	// clear flag when set
	if (alarm1 & 1)
		alarm1 &= ~0b00000001;
		rtc_write_byte(&alarm1, STATUS_ADDR);

	return alarm1 & 1 ? 1 : 0;

}
